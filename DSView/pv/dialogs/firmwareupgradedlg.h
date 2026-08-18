/*
 * CH32 firmware upgrade dialog (CDC serial IAP)
 */
#ifndef DSVIEW_PV_DIALOGS_FIRMWAREUPGRADEDLG_H
#define DSVIEW_PV_DIALOGS_FIRMWAREUPGRADEDLG_H

#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QLineEdit>
#include <QThread>
#include "dsdialog.h"

namespace pv {
namespace dialogs {

class FirmwareUpgradeWorker : public QObject
{
    Q_OBJECT
public:
    enum Mode {
        ModeUpgrade = 0,
        ModeEnterIap = 1
    };

    explicit FirmwareUpgradeWorker(Mode mode, const QString &binPath = QString(),
				   QObject *parent = 0);

public slots:
    void run();

signals:
    void progress(int percent, QString msg);
    void finished(bool ok, QString msg);

private:
    void runUpgrade();
    void runEnterIap();

    Mode _mode;
    QString _binPath;
};

class FirmwareUpgradeDlg : public DSDialog
{
    Q_OBJECT
public:
    explicit FirmwareUpgradeDlg(QWidget *parent = 0);
    ~FirmwareUpgradeDlg();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void onBrowse();
    void onStart();
    void onEnterIap();
    void onClose();
    void onProgress(int percent, QString msg);
    void onFinished(bool ok, QString msg);

private:
    bool isBusy() const;
    void setBusyUi(bool busy);
    void startWorker(FirmwareUpgradeWorker::Mode mode, const QString &binPath);

    QLineEdit *_pathEdit;
    QPushButton *_browseBtn;
    QPushButton *_enterIapBtn;
    QPushButton *_startBtn;
    QPushButton *_closeBtn;
    QProgressBar *_progress;
    QLabel *_status;
    QThread *_thread;
    FirmwareUpgradeWorker *_worker;
    bool _busy;
};

} // namespace dialogs
} // namespace pv

#endif
