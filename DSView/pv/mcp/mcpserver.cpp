#include "mcpserver.h"

#include <libsigrok.h>
#include <libsigrokdecode.h>

#include "../appcontrol.h"
#include "../sigsession.h"
#include "../deviceagent.h"
#include "../log.h"
#include "../data/snapshot.h"
#include "../data/logicsnapshot.h"
#include "../data/decode/decoder.h"
#include "../data/decode/decoderstatus.h"
#include "../data/decode/annotation.h"
#include "../data/decoderstack.h"
#include "../view/decodetrace.h"

#include <QEventLoop>
#include <QFile>
#include <QHostAddress>
#include <QJsonDocument>
#include <QSet>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTextStream>
#include <QTimer>

#include <algorithm>
#include <exception>

namespace pv {

static const char *kInstructions =
	"ALL LOGIC MCP: control the logic analyzer from an AI client.\n"
	"Typical flow: get_devices -> select_device -> configure -> "
	"add_decoder (optional) -> start_capture -> wait_capture -> "
	"get_decoder_results / export_csv.\n"
	"samplerate is in Hz. sample_count is the number of samples.\n"
	"Decoder ids are sigrok ids such as i2c, spi, uart, pwm.\n"
	"Channel maps use decoder pin names: {\"scl\":0,\"sda\":1}.";

McpServer::McpServer(QObject *parent)
	: QObject(parent)
	, _server(NULL)
	, _port(DefaultPort)
	, _running(false)
{
}

McpServer::~McpServer()
{
	stop();
}

bool McpServer::start(int port)
{
	stop();
	_server = new QTcpServer(this);
	connect(_server, SIGNAL(newConnection()), this, SLOT(on_new_connection()));

	int p = port;
	for (int i = 0; i < 10; i++) {
		if (_server->listen(QHostAddress::LocalHost, p)) {
			_port = p;
			_running = true;
			dsv_info("MCP listening on http://127.0.0.1:%d", _port);
			return true;
		}
		p = port + i + 1;
	}
	dsv_err("MCP listen failed on %d-%d", port, p);
	delete _server;
	_server = NULL;
	_running = false;
	return false;
}

void McpServer::stop()
{
	_running = false;
	if (_server) {
		_server->close();
		_server->deleteLater();
		_server = NULL;
	}
	_bufs.clear();
}

bool McpServer::is_running() const
{
	return _running && _server && _server->isListening();
}

QString McpServer::url() const
{
	return QString("http://127.0.0.1:%1").arg(_port);
}

QString McpServer::status_text() const
{
	if (is_running())
		return QString("MCP  %1").arg(url());
	return QString("MCP  offline");
}

SigSession *McpServer::session() const
{
	return AppControl::Instance()->GetSession();
}

void McpServer::on_new_connection()
{
	while (_server && _server->hasPendingConnections()) {
		QTcpSocket *sock = _server->nextPendingConnection();
		connect(sock, SIGNAL(readyRead()), this, SLOT(on_ready_read()));
		connect(sock, SIGNAL(disconnected()), this, SLOT(on_disconnected()));
		_bufs[sock] = QByteArray();
	}
}

void McpServer::on_disconnected()
{
	QTcpSocket *sock = qobject_cast<QTcpSocket *>(sender());
	if (!sock)
		return;
	_bufs.remove(sock);
	sock->deleteLater();
}

void McpServer::on_ready_read()
{
	QTcpSocket *sock = qobject_cast<QTcpSocket *>(sender());
	if (!sock)
		return;
	_bufs[sock] += sock->readAll();
	QByteArray &buf = _bufs[sock];

	for (;;) {
		int hdr = buf.indexOf("\r\n\r\n");
		if (hdr < 0)
			return;
		QByteArray head = buf.left(hdr);
		int body_len = 0;
		foreach (const QByteArray &line, head.split('\n')) {
			QByteArray l = line.trimmed().toLower();
			if (l.startsWith("content-length:"))
				body_len = l.mid(15).trimmed().toInt();
		}
		int total = hdr + 4 + body_len;
		if (buf.size() < total)
			return;
		QByteArray msg = buf.left(total);
		buf.remove(0, total);
		handle_http(sock, msg);
	}
}

void McpServer::send_http(QTcpSocket *sock, int status, const QByteArray &body,
			  const char *ctype)
{
	QByteArray reason = (status == 200) ? "OK" :
			    (status == 204) ? "No Content" : "Error";
	QByteArray hdr;
	hdr += "HTTP/1.1 " + QByteArray::number(status) + " " + reason + "\r\n";
	hdr += "Content-Type: ";
	hdr += ctype;
	hdr += "\r\n";
	hdr += "Content-Length: " + QByteArray::number(body.size()) + "\r\n";
	hdr += "Access-Control-Allow-Origin: *\r\n";
	hdr += "Access-Control-Allow-Headers: *\r\n";
	hdr += "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
	hdr += "Connection: close\r\n\r\n";
	sock->write(hdr);
	if (!body.isEmpty())
		sock->write(body);
	sock->flush();
	sock->disconnectFromHost();
}

void McpServer::send_options(QTcpSocket *sock)
{
	send_http(sock, 204, QByteArray(), "text/plain");
}

void McpServer::handle_http(QTcpSocket *sock, const QByteArray &raw)
{
	int hdr = raw.indexOf("\r\n\r\n");
	if (hdr < 0) {
		send_http(sock, 400, "bad request", "text/plain");
		return;
	}
	QByteArray head = raw.left(hdr);
	QByteArray body = raw.mid(hdr + 4);
	QList<QByteArray> lines = head.split('\n');
	if (lines.isEmpty()) {
		send_http(sock, 400, "bad request", "text/plain");
		return;
	}
	QList<QByteArray> req = lines[0].trimmed().split(' ');
	QByteArray method = req.value(0);
	QByteArray path = req.value(1);

	if (method == "OPTIONS") {
		send_options(sock);
		return;
	}

	if (method == "GET") {
		QString html = QString(
			"<!doctype html><meta charset=utf-8>"
			"<title>ALL LOGIC MCP</title>"
			"<body style='font-family:sans-serif;margin:40px'>"
			"<h1>ALL LOGIC MCP</h1>"
			"<p>Status: <b>%1</b></p>"
			"<p>POST JSON-RPC to <code>%2</code></p>"
			"<pre>claude mcp add --transport http alllogic %2\n"
			"codex mcp add --url %2 alllogic</pre>"
			"<p>Tools: get_devices, select_device, configure, "
			"start_capture, wait_capture, stop_capture, "
			"list_decoders, add_decoder, get_decoder_results, "
			"export_csv</p></body>")
			.arg(is_running() ? "running" : "offline", url());
		send_http(sock, 200, html.toUtf8(), "text/html; charset=utf-8");
		return;
	}

	if (method != "POST") {
		send_http(sock, 405, "method not allowed", "text/plain");
		return;
	}

	QJsonParseError perr;
	QJsonDocument doc = QJsonDocument::fromJson(body, &perr);
	if (perr.error != QJsonParseError::NoError || !doc.isObject()) {
		send_http(sock, 400, "invalid json", "text/plain");
		return;
	}
	QJsonObject resp = dispatch(doc.object());
	send_http(sock, 200, QJsonDocument(resp).toJson(QJsonDocument::Compact),
		  "application/json");
	(void)path;
}

QJsonObject McpServer::rpc_ok(const QJsonValue &id, const QJsonObject &result)
{
	QJsonObject o;
	o["jsonrpc"] = "2.0";
	o["id"] = id;
	o["result"] = result;
	return o;
}

QJsonObject McpServer::rpc_err(const QJsonValue &id, int code, const QString &msg)
{
	QJsonObject e;
	e["code"] = code;
	e["message"] = msg;
	QJsonObject o;
	o["jsonrpc"] = "2.0";
	o["id"] = id;
	o["error"] = e;
	return o;
}

QJsonObject McpServer::tool_text(const QString &text, bool is_error)
{
	QJsonObject item;
	item["type"] = "text";
	item["text"] = text;
	QJsonArray content;
	content.append(item);
	QJsonObject r;
	r["content"] = content;
	if (is_error)
		r["isError"] = true;
	return r;
}

QJsonObject McpServer::tool_json(const QJsonValue &value, bool is_error)
{
	QString s = QString::fromUtf8(
		QJsonDocument(value.isArray() ? QJsonDocument(value.toArray())
					      : QJsonDocument(value.toObject()))
			.toJson(QJsonDocument::Indented));
	return tool_text(s, is_error);
}

QJsonObject McpServer::dispatch(const QJsonObject &req)
{
	QJsonValue id = req.value("id");
	QString method = req.value("method").toString();
	QJsonObject params = req.value("params").toObject();

	if (method == "initialize")
		return rpc_ok(id, handle_initialize());
	if (method == "notifications/initialized" || method == "initialized") {
		QJsonObject empty;
		return rpc_ok(id.isUndefined() ? QJsonValue() : id, empty);
	}
	if (method == "ping")
		return rpc_ok(id, QJsonObject());
	if (method == "tools/list")
		return rpc_ok(id, handle_tools_list());
	if (method == "tools/call")
		return rpc_ok(id, handle_tools_call(params));
	return rpc_err(id, -32601, "Unknown method: " + method);
}

QJsonObject McpServer::handle_initialize()
{
	QJsonObject info;
	info["name"] = "alllogic";
	info["version"] = "1.4.0";
	QJsonObject caps;
	caps["tools"] = QJsonObject();
	QJsonObject r;
	r["protocolVersion"] = "2024-11-05";
	r["capabilities"] = caps;
	r["serverInfo"] = info;
	r["instructions"] = QString::fromUtf8(kInstructions);
	return r;
}

QJsonObject McpServer::tool_schema(const char *name, const char *desc,
				   const QJsonObject &props,
				   const QJsonArray &required)
{
	QJsonObject schema;
	schema["type"] = "object";
	schema["properties"] = props;
	if (!required.isEmpty())
		schema["required"] = required;
	schema["additionalProperties"] = true;
	QJsonObject t;
	t["name"] = name;
	t["description"] = desc;
	t["inputSchema"] = schema;
	return t;
}

QJsonObject McpServer::handle_tools_list()
{
	QJsonArray tools;
	tools.append(tool_schema("get_devices",
		"List connected logic analyzers and files.", QJsonObject(), QJsonArray()));
	{
		QJsonObject p;
		p["index"] = QJsonObject{{"type", "integer"},
			{"description", "Device index from get_devices"}};
		p["name"] = QJsonObject{{"type", "string"},
			{"description", "Device name substring"}};
		tools.append(tool_schema("select_device",
			"Select the active device.", p, QJsonArray()));
	}
	tools.append(tool_schema("get_status",
		"Capture and device status.", QJsonObject(), QJsonArray()));
	tools.append(tool_schema("get_channels",
		"List channels of the active device.", QJsonObject(), QJsonArray()));
	{
		QJsonObject p;
		p["samplerate_hz"] = QJsonObject{{"type", "integer"}};
		p["sample_count"] = QJsonObject{{"type", "integer"}};
		p["vth"] = QJsonObject{{"type", "number"},
			{"description", "Threshold volts"}};
		p["channel_mode"] = QJsonObject{{"type", "integer"}};
		p["operation_mode"] = QJsonObject{{"type", "string"},
			{"description", "buffer or stream"}};
		p["enabled_channels"] = QJsonObject{{"type", "array"},
			{"items", QJsonObject{{"type", "integer"}}}};
		tools.append(tool_schema("configure",
			"Set sample rate, depth, threshold, channels.", p, QJsonArray()));
	}
	{
		QJsonObject p;
		p["instant"] = QJsonObject{{"type", "boolean"}};
		tools.append(tool_schema("start_capture",
			"Start acquisition on the active device.", p, QJsonArray()));
	}
	tools.append(tool_schema("stop_capture",
		"Stop the current acquisition.", QJsonObject(), QJsonArray()));
	{
		QJsonObject p;
		p["timeout_seconds"] = QJsonObject{{"type", "number"}};
		tools.append(tool_schema("wait_capture",
			"Block until capture finishes or timeout.", p, QJsonArray()));
	}
	{
		QJsonObject p;
		p["query"] = QJsonObject{{"type", "string"},
			{"description", "Optional id/name filter"}};
		p["limit"] = QJsonObject{{"type", "integer"}};
		tools.append(tool_schema("list_decoders",
			"List protocol decoders (i2c, spi, uart, ...).", p, QJsonArray()));
	}
	{
		QJsonObject p;
		p["decoder"] = QJsonObject{{"type", "string"}};
		QJsonArray req;
		req.append("decoder");
		tools.append(tool_schema("get_decoder_info",
			"Decoder channels and options.", p, req));
	}
	{
		QJsonObject p;
		p["decoder"] = QJsonObject{{"type", "string"}};
		p["channels"] = QJsonObject{{"type", "object"},
			{"description", "Pin name to channel index, e.g. {\"scl\":0}"}};
		QJsonArray req;
		req.append("decoder");
		tools.append(tool_schema("add_decoder",
			"Attach a protocol decoder to current data.", p, req));
	}
	{
		QJsonObject p;
		p["index"] = QJsonObject{{"type", "integer"}};
		tools.append(tool_schema("remove_decoder",
			"Remove a decoder by index.", p, QJsonArray()));
	}
	{
		QJsonObject p;
		p["index"] = QJsonObject{{"type", "integer"}};
		p["max_rows"] = QJsonObject{{"type", "integer"}};
		tools.append(tool_schema("get_decoder_results",
			"Read decoder annotations.", p, QJsonArray()));
	}
	{
		QJsonObject p;
		p["path"] = QJsonObject{{"type", "string"}};
		p["max_samples"] = QJsonObject{{"type", "integer"}};
		QJsonArray req;
		req.append("path");
		tools.append(tool_schema("export_csv",
			"Export logic samples to a CSV file.", p, req));
	}

	QJsonObject r;
	r["tools"] = tools;
	return r;
}

QJsonObject McpServer::handle_tools_call(const QJsonObject &params)
{
	QString name = params.value("name").toString();
	QJsonObject args = params.value("arguments").toObject();
	try {
		if (name == "get_devices")
			return tool_get_devices();
		if (name == "select_device")
			return tool_select_device(args);
		if (name == "get_status")
			return tool_get_status();
		if (name == "get_channels")
			return tool_get_channels();
		if (name == "configure")
			return tool_configure(args);
		if (name == "start_capture")
			return tool_start_capture(args);
		if (name == "stop_capture")
			return tool_stop_capture();
		if (name == "wait_capture")
			return tool_wait_capture(args);
		if (name == "list_decoders")
			return tool_list_decoders(args);
		if (name == "get_decoder_info")
			return tool_get_decoder_info(args);
		if (name == "add_decoder")
			return tool_add_decoder(args);
		if (name == "remove_decoder")
			return tool_remove_decoder(args);
		if (name == "get_decoder_results")
			return tool_get_decoder_results(args);
		if (name == "export_csv")
			return tool_export_csv(args);
		return tool_text("Unknown tool: " + name, true);
	} catch (const std::exception &e) {
		return tool_text(QString("exception: %1").arg(e.what()), true);
	}
}

QJsonObject McpServer::tool_get_devices()
{
	struct ds_device_base_info *list = NULL;
	int count = 0;
	ds_get_device_list(&list, &count);
	int active = ds_get_actived_device_index();
	QJsonArray arr;
	for (int i = 0; i < count; i++) {
		QJsonObject o;
		o["index"] = i;
		o["name"] = QString::fromUtf8(list[i].name);
		o["active"] = (i == active);
		arr.append(o);
	}
	if (list)
		g_free(list);
	QJsonObject r;
	r["devices"] = arr;
	r["count"] = count;
	return tool_json(r);
}

QJsonObject McpServer::tool_select_device(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	if (ss->is_working())
		return tool_text("stop capture before switching device", true);

	struct ds_device_base_info *list = NULL;
	int count = 0;
	ds_get_device_list(&list, &count);
	int idx = args.contains("index") ? args.value("index").toInt(-1) : -1;
	QString name = args.value("name").toString();
	if (idx < 0 && !name.isEmpty()) {
		for (int i = 0; i < count; i++) {
			if (QString::fromUtf8(list[i].name).contains(name, Qt::CaseInsensitive)) {
				idx = i;
				break;
			}
		}
	}
	if (idx < 0 || idx >= count) {
		if (list)
			g_free(list);
		return tool_text("device not found", true);
	}
	ds_device_handle h = list[idx].handle;
	QString picked = QString::fromUtf8(list[idx].name);
	g_free(list);
	if (!ss->set_device(h))
		return tool_text("select failed: " + picked, true);
	return tool_text("selected: " + picked);
}

QJsonObject McpServer::tool_get_status()
{
	SigSession *ss = session();
	DeviceAgent *dev = ss ? ss->get_device() : NULL;
	QJsonObject o;
	o["mcp"] = url();
	o["mcp_running"] = is_running();
	o["have_device"] = dev && dev->have_instance();
	o["device"] = dev ? dev->name() : "";
	o["driver"] = dev ? dev->driver_name() : "";
	o["capturing"] = ss && ss->is_working();
	o["samplerate_hz"] = (qint64)(dev ? dev->get_sample_rate() : 0);
	o["sample_count"] = (qint64)(dev ? dev->get_sample_limit() : 0);
	o["work_mode"] = dev ? dev->get_work_mode() : -1;
	o["stream"] = dev && dev->is_stream_mode();
	if (ss) {
		data::Snapshot *snap = ss->get_snapshot(SR_CHANNEL_LOGIC);
		o["captured_samples"] = snap ? (qint64)snap->get_sample_count() : 0;
	}
	return tool_json(o);
}

QJsonObject McpServer::tool_get_channels()
{
	SigSession *ss = session();
	DeviceAgent *dev = ss ? ss->get_device() : NULL;
	if (!dev || !dev->have_instance())
		return tool_text("no active device", true);
	QJsonArray arr;
	for (GSList *l = dev->get_channels(); l; l = l->next) {
		struct sr_channel *ch = (struct sr_channel *)l->data;
		QJsonObject o;
		o["index"] = ch->index;
		o["name"] = ch->name ? QString::fromUtf8(ch->name) : "";
		o["enabled"] = (bool)ch->enabled;
		o["type"] = ch->type;
		arr.append(o);
	}
	QJsonObject r;
	r["channels"] = arr;
	return tool_json(r);
}

QJsonObject McpServer::tool_configure(const QJsonObject &args)
{
	SigSession *ss = session();
	DeviceAgent *dev = ss ? ss->get_device() : NULL;
	if (!dev || !dev->have_instance())
		return tool_text("no active device", true);
	if (ss->is_working())
		return tool_text("stop capture before configure", true);

	QStringList done;
	if (args.contains("operation_mode")) {
		QString m = args.value("operation_mode").toString().toLower();
		int v = (m == "stream") ? LO_OP_STREAM : LO_OP_BUFFER;
		dev->set_config_int16(SR_CONF_OPERATION_MODE, v);
		done << "operation_mode";
	}
	if (args.contains("channel_mode")) {
		dev->set_config_int16(SR_CONF_CHANNEL_MODE, args.value("channel_mode").toInt());
		done << "channel_mode";
	}
	if (args.contains("samplerate_hz")) {
		dev->set_config_uint64(SR_CONF_SAMPLERATE,
				       (uint64_t)args.value("samplerate_hz").toDouble());
		done << "samplerate_hz";
	}
	if (args.contains("sample_count")) {
		dev->set_config_uint64(SR_CONF_LIMIT_SAMPLES,
				       (uint64_t)args.value("sample_count").toDouble());
		done << "sample_count";
	}
	if (args.contains("vth")) {
		dev->set_config_double(SR_CONF_VTH, args.value("vth").toDouble());
		done << "vth";
	}
	if (args.contains("enabled_channels") && args.value("enabled_channels").isArray()) {
		QJsonArray want = args.value("enabled_channels").toArray();
		QSet<int> set;
		for (int i = 0; i < want.size(); i++)
			set.insert(want.at(i).toInt());
		for (GSList *l = dev->get_channels(); l; l = l->next) {
			struct sr_channel *ch = (struct sr_channel *)l->data;
			dev->enable_probe(ch, set.contains(ch->index));
		}
		done << "enabled_channels";
	}
	QJsonObject r;
	r["applied"] = QJsonArray::fromStringList(done);
	r["samplerate_hz"] = (qint64)dev->get_sample_rate();
	r["sample_count"] = (qint64)dev->get_sample_limit();
	return tool_json(r);
}

QJsonObject McpServer::tool_start_capture(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	bool instant = args.value("instant").toBool(false);
	if (!ss->start_capture(instant))
		return tool_text("start_capture failed", true);
	return tool_text("capture started");
}

QJsonObject McpServer::tool_stop_capture()
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	ss->stop_capture();
	return tool_text("capture stopped");
}

QJsonObject McpServer::tool_wait_capture(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	int timeout_ms = (int)(args.value("timeout_seconds").toDouble(60.0) * 1000.0);
	if (timeout_ms < 100)
		timeout_ms = 100;

	QEventLoop loop;
	QTimer poll;
	QTimer killer;
	poll.setInterval(100);
	QObject::connect(&poll, &QTimer::timeout, [&]() {
		if (!ss->is_working())
			loop.quit();
	});
	QObject::connect(&killer, &QTimer::timeout, &loop, &QEventLoop::quit);
	poll.start();
	killer.setSingleShot(true);
	killer.start(timeout_ms);
	loop.exec();

	bool done = !ss->is_working();
	data::Snapshot *snap = ss->get_snapshot(SR_CHANNEL_LOGIC);
	QJsonObject r;
	r["finished"] = done;
	r["capturing"] = ss->is_working();
	r["captured_samples"] = snap ? (qint64)snap->get_sample_count() : 0;
	return tool_json(r);
}

QJsonObject McpServer::tool_list_decoders(const QJsonObject &args)
{
	QString q = args.value("query").toString().toLower();
	int limit = args.value("limit").toInt(80);
	if (limit <= 0)
		limit = 80;
	QJsonArray arr;
	const GSList *l = srd_decoder_list();
	for (; l && arr.size() < limit; l = l->next) {
		const struct srd_decoder *d = (const struct srd_decoder *)l->data;
		if (!d || !d->id)
			continue;
		QString id = QString::fromUtf8(d->id);
		QString name = d->name ? QString::fromUtf8(d->name) : id;
		if (!q.isEmpty() &&
		    !id.toLower().contains(q) &&
		    !name.toLower().contains(q))
			continue;
		QJsonObject o;
		o["id"] = id;
		o["name"] = name;
		if (d->desc)
			o["desc"] = QString::fromUtf8(d->desc);
		arr.append(o);
	}
	QJsonObject r;
	r["decoders"] = arr;
	return tool_json(r);
}

QJsonObject McpServer::tool_get_decoder_info(const QJsonObject &args)
{
	QString id = args.value("decoder").toString();
	struct srd_decoder *d = srd_decoder_get_by_id(id.toUtf8().constData());
	if (!d)
		return tool_text("decoder not found: " + id, true);
	QJsonObject o;
	o["id"] = QString::fromUtf8(d->id);
	o["name"] = d->name ? QString::fromUtf8(d->name) : "";
	o["desc"] = d->desc ? QString::fromUtf8(d->desc) : "";
	QJsonArray req, opt;
	for (GSList *l = d->channels; l; l = l->next) {
		struct srd_channel *c = (struct srd_channel *)l->data;
		QJsonObject ch;
		ch["id"] = c->id ? QString::fromUtf8(c->id) : "";
		ch["name"] = c->name ? QString::fromUtf8(c->name) : "";
		req.append(ch);
	}
	for (GSList *l = d->opt_channels; l; l = l->next) {
		struct srd_channel *c = (struct srd_channel *)l->data;
		QJsonObject ch;
		ch["id"] = c->id ? QString::fromUtf8(c->id) : "";
		ch["name"] = c->name ? QString::fromUtf8(c->name) : "";
		opt.append(ch);
	}
	o["required_channels"] = req;
	o["optional_channels"] = opt;
	return tool_json(o);
}

QJsonObject McpServer::tool_add_decoder(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	QString id = args.value("decoder").toString();
	struct srd_decoder *d = srd_decoder_get_by_id(id.toUtf8().constData());
	if (!d)
		return tool_text("decoder not found: " + id, true);

	DecoderStatus *st = new DecoderStatus();
	std::list<data::decode::Decoder *> subs;
	view::Trace *trace = NULL;
	if (!ss->add_decoder(d, true, st, subs, trace)) {
		delete st;
		return tool_text("add_decoder failed", true);
	}

	auto traces = ss->get_decode_signals();
	if (traces.empty())
		return tool_text("decoder added but no trace", true);
	view::DecodeTrace *dt = traces.back();
	data::DecoderStack *stack = dt->decoder();
	if (stack && !stack->stack().empty() && args.value("channels").isObject()) {
		QJsonObject cmap = args.value("channels").toObject();
		std::map<const srd_channel *, int> probes;
		auto bind = [&](GSList *list) {
			for (GSList *l = list; l; l = l->next) {
				const srd_channel *c = (const srd_channel *)l->data;
				if (!c || !c->id)
					continue;
				QString cid = QString::fromUtf8(c->id);
				if (cmap.contains(cid))
					probes[c] = cmap.value(cid).toInt();
			}
		};
		bind(d->channels);
		bind(d->opt_channels);
		data::decode::Decoder *dec = stack->stack().front();
		dec->set_probes(probes);
		dec->commit();
	}

	QJsonObject r;
	r["index"] = (int)traces.size() - 1;
	r["decoder"] = id;
	return tool_json(r);
}

QJsonObject McpServer::tool_remove_decoder(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	int idx = args.value("index").toInt(0);
	if (idx < 0 || idx >= (int)ss->get_decode_signals().size())
		return tool_text("decoder index out of range", true);
	ss->remove_decoder(idx);
	return tool_text(QString("removed decoder %1").arg(idx));
}

QJsonObject McpServer::tool_get_decoder_results(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	auto traces = ss->get_decode_signals();
	int idx = args.contains("index") ? args.value("index").toInt() : 0;
	int max_rows = args.value("max_rows").toInt(200);
	if (max_rows <= 0)
		max_rows = 200;
	if (idx < 0 || idx >= (int)traces.size())
		return tool_text("decoder index out of range", true);

	data::DecoderStack *stack = traces[idx]->decoder();
	if (!stack)
		return tool_text("no decoder stack", true);

	QJsonArray rows;
	int nrows = stack->list_rows_size();
	for (int ri = 0; ri < nrows; ri++) {
		QString title;
		stack->list_row_title(ri, title);
		QJsonObject row;
		row["title"] = title;
		QJsonArray anns;
		uint64_t n = stack->list_annotation_size((uint16_t)ri);
		uint64_t lim = std::min<uint64_t>(n, (uint64_t)max_rows);
		for (uint64_t ci = 0; ci < lim; ci++) {
			data::decode::Annotation ann;
			if (!stack->list_annotation(&ann, (uint16_t)ri, ci))
				continue;
			QJsonObject a;
			a["start"] = (qint64)ann.start_sample();
			a["end"] = (qint64)ann.end_sample();
			QJsonArray texts;
			const std::vector<QString> &ts = ann.annotations();
			for (size_t k = 0; k < ts.size(); k++)
				texts.append(ts[k]);
			a["text"] = texts;
			anns.append(a);
		}
		row["count"] = (qint64)n;
		row["annotations"] = anns;
		rows.append(row);
	}
	QJsonObject r;
	r["index"] = idx;
	r["progress"] = stack->get_progress();
	r["running"] = stack->IsRunning();
	r["error"] = stack->error_message();
	r["rows"] = rows;
	return tool_json(r);
}

QJsonObject McpServer::tool_export_csv(const QJsonObject &args)
{
	SigSession *ss = session();
	if (!ss)
		return tool_text("no session", true);
	data::LogicSnapshot *logic =
		dynamic_cast<data::LogicSnapshot *>(ss->get_snapshot(SR_CHANNEL_LOGIC));
	if (!logic || logic->empty())
		return tool_text("no logic data", true);

	QString path = args.value("path").toString();
	if (path.isEmpty())
		return tool_text("path required", true);
	qint64 maxn = (qint64)args.value("max_samples").toDouble(200000);
	if (maxn < 1)
		maxn = 200000;

	uint64_t n = logic->get_sample_count();
	if ((qint64)n > maxn)
		n = (uint64_t)maxn;
	unsigned int nch = logic->get_channel_num();

	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return tool_text("cannot write " + path, true);
	QTextStream out(&f);
	out << "sample";
	for (unsigned int c = 0; c < nch; c++)
		out << ",D" << c;
	out << "\n";
	for (uint64_t i = 0; i < n; i++) {
		out << i;
		for (unsigned int c = 0; c < nch; c++)
			out << "," << (logic->get_sample(i, (int)c) ? 1 : 0);
		out << "\n";
	}
	f.close();

	QJsonObject r;
	r["path"] = path;
	r["samples"] = (qint64)n;
	r["channels"] = (int)nch;
	return tool_json(r);
}

} // namespace pv
