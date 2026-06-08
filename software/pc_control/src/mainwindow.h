#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QPushButton>
#include <QListWidget>
#include <QLabel>
#include "controlword.h"
#include "cdccomm.h"
#include "templatemanager.h"
#include "antennastatuspanel.h"
#include "attenuatorpanel.h"
#include "statspanel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onConnectClick();
    void onSendClick();
    void onApplyTemplate();
    void onSaveTemplate();
    void onDeleteTemplate();
    void onWordChanged();
    void onCdcData(const QByteArray &data);
    void onCdcError(const QString &error);

private:
    void buildUI();
    void buildToolbar();
    void updateFromPanels();
    void updatePanelsFromWord();

    CdccComm *m_cdc;
    QComboBox *m_portCombo;
    QPushButton *m_connectBtn;

    ControlWord m_currentWord;

    TemplateManager m_templates;
    QListWidget *m_templateList;
    QString m_templateFilePath;

    AntennaStatusPanel *m_antennaStatus;
    AttenuatorPanel *m_attenuatorPanel;
    StatsPanel *m_statsPanel;

    QLabel *m_statusLabel;

    bool m_applyingTemplate = false;
    bool m_isDark = true;
    void toggleTheme();
};

#endif // MAINWINDOW_H
