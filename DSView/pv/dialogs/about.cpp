/*
 * This file is part of the DSView project.
 * DSView is based on PulseView.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
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


#include "about.h"

#include <QPixmap>
#include <QApplication>
#include <QTextBrowser>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "../config/appconfig.h"
#include "../ui/langresource.h"

namespace pv {
namespace dialogs {

About::About(QWidget *parent) :
    DSDialog(parent, true)
{
    setFixedWidth(560);
    setMinimumHeight(420);

#if defined(__x86_64__) || defined(_M_X64)
    QString arch = "x64";
#elif defined(__i386) || defined(_M_IX86)
    QString arch = "x86";
#elif defined(__arm__) || defined(_M_ARM)
    QString arch = "arm";
#elif defined(__aarch64__)
    QString arch = "arm64";
#else
    QString arch = "other";
#endif

    const bool cn = AppConfig::Instance().IsLangCn();
    setTitle(cn ? QString::fromUtf8("关于 ALL LOGIC")
                : QString("About ALL LOGIC"));

    QLabel *icon = new QLabel(this);
    QPixmap pix(":/icons/alllogic_icon.png");
    if (!pix.isNull())
        icon->setPixmap(pix.scaled(96, 96, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    icon->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    QString html;
    if (cn) {
        html = QString::fromUtf8(
            "<div style='color:#D0D4D8; font-size:14px; line-height:1.55'>"
            "<div style='font-size:26px; color:#FFFFFF; font-weight:700'>ALL LOGIC %1</div>"
            "<div style='color:#00B4F0; margin:4px 0 14px 0'>%2 · 二次修改版</div>"
            "<p><b>本软件是对开源上位机 DSView 的二次修改。</b>"
            "我们在 DSView 源码之上自行增加了其他厂商逻辑分析仪支持、"
            "MCP 接口等功能，并更换了软件名称与图标。</p>"
            "<p>ALL LOGIC <b>不是</b> DreamSourceLab 官方产品，"
            "也<b>未获得</b>任何硬件厂商的授权或背书。"
            "设备列表中的厂商名称仅为仪器自己上报的产品名。</p>"
            "<p>原作：DreamSourceLab DSView（基于 sigrok / PulseView）<br/>"
            "本版本许可：GNU GPLv3 或更高版本（见程序目录 COPYING、NOTICE.txt）</p>"
            "<p>上游项目：<br/>"
            "DSView　<a href='https://github.com/DreamSourceLab/DSView' style='color:#00B4F0'>"
            "github.com/DreamSourceLab/DSView</a><br/>"
            "sigrok　<a href='https://sigrok.org/' style='color:#00B4F0'>sigrok.org</a></p>"
            "</div>"
            ).arg(QApplication::applicationVersion(), arch);
    } else {
        html = QString(
            "<div style='color:#D0D4D8; font-size:14px; line-height:1.55'>"
            "<div style='font-size:26px; color:#FFFFFF; font-weight:700'>ALL LOGIC %1</div>"
            "<div style='color:#00B4F0; margin:4px 0 14px 0'>%2 · secondary modification</div>"
            "<p><b>ALL LOGIC is a secondary modification of the open-source host DSView.</b> "
            "We added community drivers for additional logic analyzers, an MCP interface, "
            "and a new name and icon on top of the DSView source tree.</p>"
            "<p>ALL LOGIC is <b>not</b> an official DreamSourceLab product and is "
            "<b>not</b> licensed or endorsed by any hardware vendor. "
            "Vendor names in the device list are reported by the instruments themselves.</p>"
            "<p>Original work: DreamSourceLab DSView (based on sigrok / PulseView)<br/>"
            "This build: GNU GPLv3 or later (see COPYING and NOTICE.txt)</p>"
            "<p>Upstream:<br/>"
            "DSView　<a href='https://github.com/DreamSourceLab/DSView' style='color:#00B4F0'>"
            "github.com/DreamSourceLab/DSView</a><br/>"
            "sigrok　<a href='https://sigrok.org/' style='color:#00B4F0'>sigrok.org</a></p>"
            "</div>"
            ).arg(QApplication::applicationVersion(), arch);
    }

    QTextBrowser *body = new QTextBrowser(this);
    body->setOpenExternalLinks(true);
    body->setFrameStyle(QFrame::NoFrame);
    body->setHtml(html);

    QHBoxLayout *head = new QHBoxLayout();
    head->setContentsMargins(0, 0, 0, 8);
    head->addWidget(icon);
    head->addStretch();

    QVBoxLayout *box = new QVBoxLayout();
    box->addLayout(head);
    box->addWidget(body);
    layout()->addLayout(box);
}

About::~About()
{
}

} // namespace dialogs
} // namespace pv
