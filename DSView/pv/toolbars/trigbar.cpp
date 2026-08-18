/*
 * This file is part of the DSView project.
 * DSView is based on PulseView.
 *
 * Copyright (C) 2013 DreamSourceLab <support@dreamsourcelab.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "trigbar.h"

#include <QBitmap>
#include <QPainter>
#include <QEvent>

#include "../sigsession.h"
#include "../dialogs/fftoptions.h"
#include "../dialogs/lissajousoptions.h"
#include "../dialogs/mathoptions.h"
#include "../dialogs/firmwareupgradedlg.h"
#include "../view/trace.h"
#include "../dialogs/applicationpardlg.h"
#include "../dialogs/dsdialog.h"
#include "../ui/langresource.h"
#include "../config/appconfig.h"
#include "../appcontrol.h"
#include "../mcp/mcpserver.h"
#include "../ui/fn.h"
#include "../ui/xspinbox.h"

#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace pv {
namespace toolbars {
 
TrigBar::TrigBar(SigSession *session, QWidget *parent) :
    QToolBar("Trig Bar", parent),
    _session(session),
 
    _trig_button(this),
    _protocol_button(this),
    _measure_button(this),
    _upgrade_button(this),
    _function_button(this),
    _setting_button(this)
{
    _enable = true;

    setMovable(false);
    setContentsMargins(0,0,0,0);

    _action_fft = new QAction(this);
    _action_fft->setObjectName(QString::fromUtf8("actionFft"));
   
    _action_math = new QAction(this);
    _action_math->setObjectName(QString::fromUtf8("actionMath"));
     
    _function_menu = new QMenu(this);
    _function_menu->setContentsMargins(0,0,0,0);
    _function_menu->addAction(_action_fft);
    _function_menu->addAction(_action_math);
    _function_button.setPopupMode(QToolButton::InstantPopup);
    _function_button.setMenu(_function_menu);

    _action_lissajous = new QAction(this);
    _action_lissajous->setObjectName(QString::fromUtf8("actionLissajous"));
   
    _dark_style = new QAction(this);
    _dark_style->setObjectName(QString::fromUtf8("actionDark"));
    
    _light_style = new QAction(this);
    _light_style->setObjectName(QString::fromUtf8("actionLight"));
     
    _themes = new QMenu(this);
    _themes->setObjectName(QString::fromUtf8("menuThemes"));
    _themes->addAction(_light_style);
    _themes->addAction(_dark_style);

     _action_dispalyOptions = new QAction(this);
    _action_mcpOptions = new QAction(this);

    _display_menu = new QMenu(this);
    _display_menu->setContentsMargins(0,0,0,0);

    _display_menu->addAction(_action_mcpOptions);
    _display_menu->addSeparator();
    _display_menu->addAction(_action_lissajous);    
    _display_menu->addMenu(_themes);
	_display_menu->addAction(_action_dispalyOptions);

    _setting_button.setPopupMode(QToolButton::InstantPopup);
    _setting_button.setMenu(_display_menu);

    _trig_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    _protocol_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    _measure_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    _upgrade_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    _function_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    _setting_button.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    _protocol_button.setContentsMargins(0,0,0,0);

    _trig_action = addWidget(&_trig_button);
    _protocol_action = addWidget(&_protocol_button);
    _measure_action = addWidget(&_measure_button);
    _upgrade_action = addWidget(&_upgrade_button);
    _function_action = addWidget(&_function_button); 
    _display_action = addWidget(&_setting_button); //must be created

    connect(&_trig_button, SIGNAL(clicked()),this, SLOT(trigger_clicked()));
    connect(&_protocol_button, SIGNAL(clicked()),this, SLOT(protocol_clicked()));
    connect(&_measure_button, SIGNAL(clicked()),this, SLOT(measure_clicked()));
    connect(&_upgrade_button, SIGNAL(clicked()), this, SLOT(upgrade_clicked()));

    connect(_action_fft, SIGNAL(triggered()), this, SLOT(on_actionFft_triggered()));
    connect(_action_math, SIGNAL(triggered()), this, SLOT(on_actionMath_triggered()));
    connect(_action_lissajous, SIGNAL(triggered()), this, SLOT(on_actionLissajous_triggered()));
    connect(_dark_style, SIGNAL(triggered()), this, SLOT(on_actionDark_triggered()));
    connect(_light_style, SIGNAL(triggered()), this, SLOT(on_actionLight_triggered()));
    connect(_action_dispalyOptions, SIGNAL(triggered()), this, SLOT(on_display_setting()));
    connect(_action_mcpOptions, SIGNAL(triggered()), this, SLOT(on_mcp_setting()));

    ADD_UI(this);
}

TrigBar::~TrigBar()
{
    REMOVE_UI(this);
}

void TrigBar::retranslateUi()
{
    _trig_button.setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_TRIGGER), "Trigger"));
    _protocol_button.setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DECODE), "Decode"));
    _measure_button.setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_MEASURE), "Measure"));
    _upgrade_button.setText(QString::fromUtf8("固件升级"));
    _function_button.setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_FUNCTION), "Function"));

    _setting_button.setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY), "MCP"));
    _themes->setTitle(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY_THEMES), "Themes"));
    _action_lissajous->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY_LISSAJOUS), "Lissajous"));

   
    _dark_style->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY_THEMES_DARK), "Dark"));
    _light_style->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY_THEMES_LIGHT), "Light"));

    _action_fft->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_FUNCTION_FFT), "FFT"));
    _action_math->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_FUNCTION_MATH), "Math"));

    _action_dispalyOptions->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_DISPLAY_OPTIONS), "Options"));
    _action_mcpOptions->setText(L_S(STR_PAGE_TOOLBAR, S_ID(IDS_TOOLBAR_MCP_OPTIONS), "MCP Settings"));
}

void TrigBar::reStyle()
{
    QString iconPath = GetIconPath();

    _trig_button.setIcon(QIcon(iconPath+"/trigger.svg"));
    _protocol_button.setIcon(QIcon(iconPath+"/protocol.svg"));
    _measure_button.setIcon(QIcon(iconPath+"/measure.svg"));
    _upgrade_button.setIcon(QIcon(iconPath+"/update.svg"));
    _function_button.setIcon(QIcon(iconPath+"/function.svg"));
    _setting_button.setIcon(QIcon(iconPath+"/display.svg"));

    _action_fft->setIcon(QIcon(iconPath+"/fft.svg"));
    _action_math->setIcon(QIcon(iconPath+"/math.svg"));
    _action_lissajous->setIcon(QIcon(iconPath+"/lissajous.svg"));
    _dark_style->setIcon(QIcon(iconPath+"/dark.svg"));
    _light_style->setIcon(QIcon(iconPath+"/light.svg"));

    _action_dispalyOptions->setIcon(QIcon(iconPath+"/gear.svg"));
    _action_mcpOptions->setIcon(QIcon(iconPath+"/gear.svg"));

     AppConfig &app = AppConfig::Instance();
     QString icon_fname = iconPath +"/"+ app.frameOptions.style +".svg";  
    _themes->setIcon(QIcon(icon_fname));
}

void TrigBar::protocol_clicked()
{  
    if (_protocol_button.isVisible() && _protocol_button.isEnabled())
    {
        DockOptions *opt = getDockOptions();
        opt->decodeDock = !opt->decodeDock;
        sig_protocol(opt->decodeDock);
        AppConfig::Instance().SaveFrame();

        update_checked_status();
    }
}

void TrigBar::trigger_clicked()
{
    if (_trig_button.isVisible() && _trig_button.isEnabled())
    {
        DockOptions *opt = getDockOptions();
        opt->triggerDock = !opt->triggerDock;
        sig_trigger(opt->triggerDock);
        AppConfig::Instance().SaveFrame();

        update_checked_status();
    }
}

void TrigBar::measure_clicked()
{   
    if (_measure_button.isVisible() && _measure_button.isEnabled())
    {
        DockOptions *opt = getDockOptions();
        opt->measureDock = !opt->measureDock;
        sig_measure(opt->measureDock);
        AppConfig::Instance().SaveFrame();

        update_checked_status();
    }
}

void TrigBar::upgrade_clicked()
{
    if (_upgrade_button.isVisible() && _upgrade_button.isEnabled())
    {
        pv::dialogs::FirmwareUpgradeDlg dlg(this);
        dlg.exec();
    }
}

void TrigBar::reload()
{ 
    int mode = _session->get_device()->get_work_mode();

    if (mode == LOGIC) {
        _trig_action->setVisible(true);
        _protocol_action->setVisible(true);
        _measure_action->setVisible(true);
        _upgrade_action->setVisible(true);
        _function_action->setVisible(false);
        _action_lissajous->setVisible(false);
        _action_dispalyOptions->setVisible(true);

    } else if (mode == ANALOG) {
        _trig_action->setVisible(false);
        _protocol_action->setVisible(false);
        _measure_action->setVisible(true);
        _upgrade_action->setVisible(true);
        _function_action->setVisible(false);
        _action_lissajous->setVisible(false);
        _action_dispalyOptions->setVisible(true);

    } else if (mode == DSO) {
        _trig_action->setVisible(true);
        _protocol_action->setVisible(false);
        _measure_action->setVisible(true);
        _upgrade_action->setVisible(true);
        _function_action->setVisible(true);
        _action_lissajous->setVisible(true);
        _action_dispalyOptions->setVisible(true);
    }

    DockOptions *opt = getDockOptions();

    bool bDecoder = _protocol_action->isVisible() && opt->decodeDock;
    bool bTrigger = _trig_action->isVisible() && opt->triggerDock;
    bool bMeasure = _measure_action->isVisible() && opt->measureDock;

    sig_protocol(bDecoder);
    sig_trigger(bTrigger);
    sig_measure(bMeasure);
   
    update_view_status(); 
    update_checked_status();
    update();    
}

void TrigBar::on_actionFft_triggered()
{
    pv::dialogs::FftOptions fft_dlg(this, _session);
    fft_dlg.exec();
}

void TrigBar::on_actionMath_triggered()
{
    pv::dialogs::MathOptions math_dlg(_session, this);
    if (math_dlg.exec() == QDialog::Accepted)
    {
        math_dlg.Apply();
    }
}

void TrigBar::on_actionDark_triggered()
{
    sig_setTheme(THEME_STYLE_DARK);
    QString icon = GetIconPath() + "/" + THEME_STYLE_DARK + ".svg";
    _themes->setIcon(QIcon(icon));
}

void TrigBar::on_actionLight_triggered()
{
    sig_setTheme(THEME_STYLE_LIGHT);
    QString icon = GetIconPath() + "/" + THEME_STYLE_LIGHT +".svg";
    _themes->setIcon(QIcon(icon));
}

void TrigBar::on_actionLissajous_triggered()
{
    pv::dialogs::LissajousOptions lissajous_dlg(_session, this);
    lissajous_dlg.exec();
}

 void TrigBar::on_display_setting()
 {    
    pv::dialogs::ApplicationParamDlg dlg;
    dlg.ShowDlg(this);
 }

void TrigBar::on_mcp_setting()
{
    AppConfig &app = AppConfig::Instance();
    AppControl *ctl = AppControl::Instance();
    pv::McpServer *mcp = ctl->GetMcp();

    dialogs::DSDialog dlg(this, false, true);
    dlg.setTitle(L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_OPTIONS), "MCP Settings"));
    dlg.setMinimumSize(420, 220);

    QWidget *panel = new QWidget(&dlg);
    dlg.layout()->addWidget(panel);
    QFormLayout *lay = new QFormLayout();
    panel->setLayout(lay);
    lay->setVerticalSpacing(12);

    QCheckBox *ckEnable = new QCheckBox();
    ckEnable->setChecked(app.appOptions.mcpEnabled);
    lay->addRow(L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_ENABLE), "Enable MCP"), ckEnable);

    XSpinBox *spPort = new XSpinBox(panel);
    spPort->setRange(1024, 65535);
    spPort->setValue(app.appOptions.mcpPort);
    lay->addRow(L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_PORT), "Port"), spPort);

    QLabel *lbStatus = new QLabel(ctl->McpStatusText());
    lay->addRow(L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_STATUS), "Status"), lbStatus);

    QWidget *urlRow = new QWidget();
    QHBoxLayout *urlLay = new QHBoxLayout();
    urlLay->setContentsMargins(0, 0, 0, 0);
    urlRow->setLayout(urlLay);
    QLineEdit *edUrl = new QLineEdit();
    edUrl->setReadOnly(true);
    edUrl->setText(mcp ? mcp->url() : QString("http://127.0.0.1:%1").arg(app.appOptions.mcpPort));
    QPushButton *btCopy = new QPushButton(
        L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_COPY), "Copy"));
    urlLay->addWidget(edUrl);
    urlLay->addWidget(btCopy);
    lay->addRow(L_S(STR_PAGE_DLG, S_ID(IDS_DLG_MCP_URL), "URL"), urlRow);

    connect(btCopy, &QPushButton::clicked, [edUrl]() {
        QApplication::clipboard()->setText(edUrl->text());
    });

    dlg.exec();
    if (!dlg.IsClickYes())
        return;

    app.appOptions.mcpEnabled = ckEnable->isChecked();
    app.appOptions.mcpPort = spPort->value();
    app.SaveApp();
    ctl->ApplyMcp(app.appOptions.mcpEnabled, app.appOptions.mcpPort);
}

 DockOptions* TrigBar::getDockOptions()
 {
    AppConfig &app = AppConfig::Instance(); 
    int mode = _session->get_device()->get_work_mode();

    if (mode == LOGIC)
        return &app.frameOptions._logicDock;
     else if (mode == DSO)
        return &app.frameOptions._dsoDock;
    else
        return &app.frameOptions._analogDock;
 }

 void TrigBar::update_view_status()
 {
    bool bEnable = _session->is_working() == false;

    _trig_button.setEnabled(bEnable);
    _protocol_button.setEnabled(bEnable);
    _measure_button.setEnabled(bEnable);
    _upgrade_button.setEnabled(bEnable);
    _function_button.setEnabled(bEnable);
    _setting_button.setEnabled(bEnable);

    if (_session->is_working() && _session->get_device()->get_work_mode() == DSO){
        if (_session->is_instant() == false){
            _trig_button.setEnabled(true);
            _measure_button.setEnabled(true);
            _function_button.setEnabled(true);
            _setting_button.setEnabled(true);
        }
    }
 }

void TrigBar::update_checked_status()
{
    DockOptions *opt = getDockOptions();
    assert(opt);

    _trig_button.setCheckable(true); 
    _protocol_button.setCheckable(true); 
    _measure_button.setCheckable(true);
    _upgrade_button.setCheckable(false);

    _trig_button.setChecked(opt->triggerDock);
    _protocol_button.setChecked(opt->decodeDock);
    _measure_button.setChecked(opt->measureDock);
}

void TrigBar::UpdateLanguage()
{
    retranslateUi();
}

void TrigBar::UpdateTheme()
{
    reStyle();
}

void TrigBar::UpdateFont()
{ 
    QFont font = this->font();
    font.setPointSizeF(AppConfig::Instance().appOptions.fontSize);
    ui::set_toolbar_font(this, font);
}

} // namespace toolbars
} // namespace pv
