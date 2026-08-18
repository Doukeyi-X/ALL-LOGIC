/*
 * Built-in MCP (Model Context Protocol) HTTP server.
 * Independent of PXView. JSON-RPC 2.0 over HTTP on localhost.
 */
#ifndef DSVIEW_PV_MCP_MCPSERVER_H
#define DSVIEW_PV_MCP_MCPSERVER_H

#include <QObject>
#include <QByteArray>
#include <QHash>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QString>

class QTcpServer;
class QTcpSocket;

namespace pv {

class SigSession;

class McpServer : public QObject
{
	Q_OBJECT

public:
	static const int DefaultPort = 10110;

	explicit McpServer(QObject *parent = 0);
	~McpServer();

	bool start(int port = DefaultPort);
	void stop();
	bool is_running() const;
	int port() const { return _port; }
	QString url() const;
	QString status_text() const;

private slots:
	void on_new_connection();
	void on_ready_read();
	void on_disconnected();

private:
	void handle_http(QTcpSocket *sock, const QByteArray &raw);
	void send_http(QTcpSocket *sock, int status, const QByteArray &body,
		       const char *ctype);
	void send_options(QTcpSocket *sock);

	QJsonObject dispatch(const QJsonObject &req);
	QJsonObject rpc_ok(const QJsonValue &id, const QJsonObject &result);
	QJsonObject rpc_err(const QJsonValue &id, int code, const QString &msg);
	QJsonObject tool_text(const QString &text, bool is_error = false);
	QJsonObject tool_json(const QJsonValue &value, bool is_error = false);

	QJsonObject handle_initialize();
	QJsonObject handle_tools_list();
	QJsonObject handle_tools_call(const QJsonObject &params);

	QJsonObject tool_get_devices();
	QJsonObject tool_select_device(const QJsonObject &args);
	QJsonObject tool_get_status();
	QJsonObject tool_get_channels();
	QJsonObject tool_configure(const QJsonObject &args);
	QJsonObject tool_start_capture(const QJsonObject &args);
	QJsonObject tool_stop_capture();
	QJsonObject tool_wait_capture(const QJsonObject &args);
	QJsonObject tool_list_decoders(const QJsonObject &args);
	QJsonObject tool_get_decoder_info(const QJsonObject &args);
	QJsonObject tool_add_decoder(const QJsonObject &args);
	QJsonObject tool_remove_decoder(const QJsonObject &args);
	QJsonObject tool_get_decoder_results(const QJsonObject &args);
	QJsonObject tool_export_csv(const QJsonObject &args);

	SigSession *session() const;
	static QJsonObject tool_schema(const char *name, const char *desc,
				       const QJsonObject &props,
				       const QJsonArray &required);

	QTcpServer *_server;
	QHash<QTcpSocket *, QByteArray> _bufs;
	int _port;
	bool _running;
};

} // namespace pv

#endif
