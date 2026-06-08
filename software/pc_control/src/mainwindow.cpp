#include "mainwindow.h"
#include "themedata.h"
#include <QToolBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QInputDialog>
#include <QMessageBox>
#include <QRegularExpression>
#include <QApplication>
#include <QGuiApplication>
#include <QScreen>
#include <QDir>
#include <QStyle>
#include <QSettings>
#include <QStandardPaths>
#include <QDialog>
#include <QLineEdit>
#include <QDialogButtonBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_cdc(new CdccComm(this))
{
    setWindowTitle(tr("射频前端控制系统"));

    // 数据存放于 %APPDATA%/rf_frontend_control/
    QString dataDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/rf_frontend_control";
    QDir().mkpath(dataDir);
    m_templateFilePath = dataDir + "/templates.json";

    // 记忆上次主题
    QSettings settings(dataDir + "/settings.ini", QSettings::IniFormat);
    m_isDark = settings.value("theme/dark", true).toBool();
    currentTheme = m_isDark ? &themeDark : &themeLight;
    setStyleSheet(currentTheme->toStyleSheet());

    // 窗口尺寸: 默认 1600x900 (16:9), 最小 1280x720, 不超过屏幕 95%
    QScreen *screen = QApplication::primaryScreen();
    QRect sg = screen->availableGeometry();
    int maxW = static_cast<int>(sg.width() * 0.95);
    int maxH = static_cast<int>(sg.height() * 0.95);

    int wantW = qMin(1600, maxW);
    int wantH = qMin(900, maxH);
    resize(wantW, wantH);
    setMinimumSize(1280, 720);

    buildUI();
    buildToolbar();

    // 优先加载用户模板；无文件时用内置默认
    if (QFile::exists(m_templateFilePath)) {
        m_templates.loadFromFile(m_templateFilePath);
        // 确保 STANDBY 始终存在且不可修改
        int stIdx = m_templates.findByName("STANDBY");
        if (stIdx >= 0)
            m_templates.at(stIdx).builtIn = true;
        else {
            ControlWord cw;
            for (int m = 1; m <= 3; m++)
                for (int c = 1; c <= 3; c++) {
                    cw.setIFBand(m, c, false);
                    cw.setIFAttenuation(m, c, 15);
                }
            cw.markAllDirty();
            m_templates.addBuiltinTemplate("STANDBY", "天线全关，IF 衰减最大 + 窄带", cw);
        }
    } else {
        m_templates.loadDefaults();
    }

    for (int i = 0; i < m_templates.count(); i++)
        m_templateList->addItem(m_templates.at(i).name);

    // 启动时默认 STANDBY 状态（用 m_applyingTemplate 防止级联污染）
    m_currentWord.applyDirtyFrom(m_templates.at(0).word);
    m_applyingTemplate = true;
    updatePanelsFromWord();
    m_applyingTemplate = false;
    m_templateList->setCurrentRow(0);

    m_statusLabel->setText(tr("未连接"));

    connect(m_cdc, &CdccComm::dataReceived, this, &MainWindow::onCdcData);
    connect(m_cdc, &CdccComm::errorOccurred, this, &MainWindow::onCdcError);
    connect(m_cdc, &CdccComm::connected, this, [this]() {
        m_statusLabel->setText(tr("已连接: %1").arg(m_cdc->currentPort()));
        m_statusLabel->setProperty("connected", true);
        m_statusLabel->style()->unpolish(m_statusLabel);
        m_statusLabel->style()->polish(m_statusLabel);
        m_connectBtn->setText(tr("断开"));
    });
    connect(m_cdc, &CdccComm::disconnected, this, [this]() {
        m_statusLabel->setText(tr("未连接"));
        m_statusLabel->setProperty("connected", false);
        m_statusLabel->style()->unpolish(m_statusLabel);
        m_statusLabel->style()->polish(m_statusLabel);
        m_connectBtn->setText(tr("连接"));
    });
}

MainWindow::~MainWindow() {
    m_cdc->close();
}

void MainWindow::buildUI() {
    QWidget *centralWidget = new QWidget();
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(4, 4, 4, 4);

    QSplitter *splitter = new QSplitter(Qt::Horizontal);
    splitter->setHandleWidth(5);

    // ---- 左栏: 天线控制 ----
    m_antennaStatus = new AntennaStatusPanel();
    m_antennaStatus->setMinimumWidth(460);
    m_antennaStatus->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    splitter->addWidget(m_antennaStatus);

    // ---- 中栏: 信号处理 ----
    m_attenuatorPanel = new AttenuatorPanel();
    m_attenuatorPanel->setMinimumWidth(380);
    m_attenuatorPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    splitter->addWidget(m_attenuatorPanel);

    // ---- 右栏: 一个大框包含状态+模板+手动输入 ----
    QGroupBox *rightPanel = new QGroupBox(tr("状态监视"));
    rightPanel->setObjectName("rightPanel");
    rightPanel->setMinimumWidth(360);
    rightPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    QVBoxLayout *rpLayout = new QVBoxLayout(rightPanel);
    rpLayout->setSpacing(4);
    rpLayout->setContentsMargins(0, 0, 0, 0);

    m_statsPanel = new StatsPanel();
    m_statsPanel->setObjectName("statsPanel");
    m_statsPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    rpLayout->addWidget(m_statsPanel, 1);

    QLabel *tmplLabel = new QLabel(tr("模板管理"));
    tmplLabel->setObjectName("sectionTitle");
    rpLayout->addWidget(tmplLabel);
    m_templateList = new QListWidget();
    m_templateList->setFixedHeight(120);
    rpLayout->addWidget(m_templateList);

    QHBoxLayout *tmplBtns = new QHBoxLayout();
    tmplBtns->setSpacing(4);
    QPushButton *applyBtn = new QPushButton(tr("应用"));
    QPushButton *saveBtn = new QPushButton(tr("修改"));
    QPushButton *delBtn = new QPushButton(tr("删除"));
    tmplBtns->addWidget(applyBtn);
    tmplBtns->addWidget(saveBtn);
    tmplBtns->addWidget(delBtn);
    rpLayout->addLayout(tmplBtns);

    connect(applyBtn, &QPushButton::clicked, this, &MainWindow::onApplyTemplate);
    connect(saveBtn, &QPushButton::clicked, this, &MainWindow::onSaveTemplate);
    connect(delBtn, &QPushButton::clicked, this, &MainWindow::onDeleteTemplate);
    connect(m_templateList, &QListWidget::itemDoubleClicked, this, &MainWindow::onApplyTemplate);

    splitter->addWidget(rightPanel);

    // 初始宽度比例近似 1 : 1.1 : 1
    splitter->setSizes(QList<int>() << 480 << 520 << 480);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    splitter->setStretchFactor(2, 1);

    mainLayout->addWidget(splitter);
    setCentralWidget(centralWidget);

    connect(m_antennaStatus, &AntennaStatusPanel::changed, this, &MainWindow::onWordChanged);
    connect(m_attenuatorPanel, &AttenuatorPanel::changed, this, &MainWindow::onWordChanged);

    connect(m_statsPanel->sendButton(), &QPushButton::clicked, this, &MainWindow::onSendClick);
    connect(m_statsPanel->saveTemplateButton(), &QPushButton::clicked, this, [this]() {
        std::array<uint8_t, 12> bytes{};
        if (!m_statsPanel->parseHexBytes(bytes)) {
            QMessageBox::warning(this, tr("错误"), tr("hex 输入无效"));
            return;
        }
        bool ok;
        QString name = QInputDialog::getText(this, tr("保存模板"),
            tr("模板名称:"), QLineEdit::Normal, "", &ok);
        if (!ok || name.isEmpty()) return;
        if (name == "STANDBY") { name += "_副本"; }
        ControlWord w = ControlWord::fromSendBytes(bytes);
        w.markAllDirty();
        if (m_templates.findByName(name) >= 0)
            name += "_副本";
        m_templates.addTemplate(name, tr("用户自定义"), w);
        m_templateList->addItem(name);
        m_templates.saveToFile(m_templateFilePath);
        m_statsPanel->appendLog(tr("保存模板: %1").arg(name));
    });
}

void MainWindow::buildToolbar() {
    QToolBar *toolbar = addToolBar(tr("通信"));
    toolbar->setMovable(false);
    // 工具栏样式由全局 QSS 统一定义

    m_portCombo = new QComboBox();
    m_portCombo->setMinimumWidth(120);
    m_portCombo->addItems(CdccComm::availablePorts());
    QPushButton *refreshBtn = new QPushButton(tr("刷新"));
    m_connectBtn = new QPushButton(tr("连接"));
    toolbar->addWidget(new QLabel(tr(" COM口: ")));
    toolbar->addWidget(m_portCombo);
    toolbar->addWidget(refreshBtn);
    toolbar->addWidget(m_connectBtn);

    m_statusLabel = new QLabel(tr("未连接"));
    m_statusLabel->setObjectName("statusLabel");
    toolbar->addSeparator();
    toolbar->addWidget(m_statusLabel);

    toolbar->addSeparator();
    QPushButton *themeBtn = new QPushButton("☀");
    themeBtn->setObjectName("themeBtn");
    themeBtn->setCheckable(true);
    themeBtn->setToolTip(tr("切换亮/暗主题"));
    connect(themeBtn, &QPushButton::clicked, this, &MainWindow::toggleTheme);
    toolbar->addWidget(themeBtn);

    connect(refreshBtn, &QPushButton::clicked, this, [this]() {
        m_portCombo->clear();
        m_portCombo->addItems(CdccComm::availablePorts());
    });
    connect(m_connectBtn, &QPushButton::clicked, this, &MainWindow::onConnectClick);
}

void MainWindow::onConnectClick() {
    if (m_cdc->isOpen()) {
        m_cdc->close();
    } else {
        QString port = m_portCombo->currentText();
        if (port.isEmpty()) {
            QMessageBox::warning(this, tr("错误"), tr("请选择 COM 口"));
            return;
        }
        if (!m_cdc->open(port))
            QMessageBox::warning(this, tr("错误"), tr("无法打开 %1").arg(port));
    }
}

void MainWindow::onSendClick() {
    if (!m_cdc->isOpen()) {
        QMessageBox::warning(this, tr("错误"), tr("请先连接 CDC 设备"));
        return;
    }
    std::array<uint8_t, 12> bytes{};
    if (!m_statsPanel->parseHexBytes(bytes)) {
        QMessageBox::warning(this, tr("错误"), tr("hex 输入无效，请输入 12 字节 hex 值"));
        return;
    }
    QString hex;
    for (auto b : bytes) hex += QString("%1 ").arg(b, 2, 16, QChar('0'));
    m_statsPanel->appendLog(tr("发送: %1").arg(hex.toUpper()));
    m_cdc->sendTemplate(bytes);
}

void MainWindow::onWordChanged() {
    if (!m_applyingTemplate)
        updateFromPanels();
    m_statsPanel->updateDisplay(m_currentWord);
    m_antennaStatus->loadFrom(m_currentWord);
}

void MainWindow::updateFromPanels() {
    m_currentWord.reset();
    m_antennaStatus->applyTo(m_currentWord);
    m_attenuatorPanel->applyTo(m_currentWord);
}

void MainWindow::updatePanelsFromWord() {
    m_antennaStatus->loadFrom(m_currentWord);
    m_attenuatorPanel->loadFrom(m_currentWord);
    m_statsPanel->updateDisplay(m_currentWord);
}

void MainWindow::onSaveTemplate() {
    int idx = m_templateList->currentRow();
    if (idx < 0 || idx >= m_templates.count()) return;
    if (m_templates.at(idx).builtIn || m_templates.at(idx).name == "STANDBY") {
        QMessageBox::information(this, tr("提示"), tr("系统内置模板不可修改"));
        return;
    }

    // 弹窗显示模板名和 hex 控制码，允许修改
    auto tpl = m_templates.at(idx).word.rawBytes();
    QString hex;
    for (auto b : tpl) hex += QString("%1 ").arg(b, 2, 16, QChar('0'));

    QDialog dlg(this);
    dlg.setWindowTitle(tr("修改模板"));
    dlg.setMinimumWidth(480);
    QVBoxLayout *vl = new QVBoxLayout(&dlg);

    vl->addWidget(new QLabel(tr("模板名:")));
    QLineEdit *nameEdit = new QLineEdit(m_templates.at(idx).name);
    nameEdit->setFont(QFont("Consolas", 14));
    nameEdit->setPlaceholderText(tr("模板名称"));
    vl->addWidget(nameEdit);

    vl->addWidget(new QLabel(tr("控制码:")));
    QLineEdit *hexEdit = new QLineEdit(hex.toUpper().trimmed());
    hexEdit->setFont(QFont("Consolas", 14));
    hexEdit->setMinimumWidth(380);
    hexEdit->setPlaceholderText(tr("12 字节 hex，空格分隔"));
    vl->addWidget(hexEdit);

    QDialogButtonBox *btns = new QDialogButtonBox;
    btns->addButton(tr("确认"), QDialogButtonBox::AcceptRole);
    btns->addButton(tr("取消"), QDialogButtonBox::RejectRole);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    QString newName = nameEdit->text().trimmed();
    if (newName.isEmpty()) {
        QMessageBox::warning(this, tr("错误"), tr("模板名不能为空"));
        return;
    }

    QStringList parts = hexEdit->text().trimmed().split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() != 12) {
        QMessageBox::warning(this, tr("错误"), tr("请填入 12 字节 hex 值"));
        return;
    }
    bool ok;
    std::array<uint8_t, 12> newBytes{};
    for (int i = 0; i < 12; i++) {
        newBytes[i] = static_cast<uint8_t>(parts[i].toUInt(&ok, 16));
        if (!ok) {
            QMessageBox::warning(this, tr("错误"), tr("无效 hex 值: %1").arg(parts[i]));
            return;
        }
    }

    // 更新模板名（列表同步）
    QString oldName = m_templates.at(idx).name;
    if (newName != oldName) {
        if (m_templates.findByName(newName) >= 0) {
            QMessageBox::warning(this, tr("错误"), tr("已存在同名模板"));
            return;
        }
        m_templates.at(idx).name = newName;
        m_templateList->item(idx)->setText(newName);
    }
    m_templates.at(idx).word = ControlWord::fromSendBytes(newBytes);
    m_templates.at(idx).word.markAllDirty();
    m_templates.saveToFile(m_templateFilePath);
    m_statsPanel->appendLog(tr("已修改模板: %1").arg(m_templates.at(idx).name));
}

void MainWindow::onApplyTemplate() {
    int idx = m_templateList->currentRow();
    if (idx < 0 || idx >= m_templates.count()) return;
    m_currentWord.applyDirtyFrom(m_templates.at(idx).word);
    m_applyingTemplate = true;
    updatePanelsFromWord();
    m_applyingTemplate = false;
    m_statsPanel->appendLog(tr("应用模板: %1").arg(m_templates.at(idx).name));
}

void MainWindow::onDeleteTemplate() {
    int idx = m_templateList->currentRow();
    if (idx < 0 || idx >= m_templates.count()) return;
    if (m_templates.at(idx).builtIn) {
        QMessageBox::information(this, tr("提示"), tr("系统内置模板不可删除"));
        return;
    }
    if (QMessageBox::question(this, tr("确认"),
            tr("删除模板 '%1'?").arg(m_templates.at(idx).name))
        != QMessageBox::Yes) return;
    m_templates.removeTemplate(idx);
    delete m_templateList->takeItem(idx);
    m_templates.saveToFile(m_templateFilePath);
}

void MainWindow::onCdcData(const QByteArray &data) {
    QString ascii;
    for (auto b : data)
        ascii += (b >= 32 && b < 127) ? QChar(b) : '.';
    m_statsPanel->appendLog(tr("收到: %1").arg(ascii));
}

void MainWindow::onCdcError(const QString &error) {
    m_statsPanel->appendLog(tr("错误: %1").arg(error));
    m_statusLabel->setText(tr("错误: %1").arg(error));
}

void MainWindow::toggleTheme() {
    m_isDark = !m_isDark;
    currentTheme = m_isDark ? &themeDark : &themeLight;
    setStyleSheet(currentTheme->toStyleSheet());
    m_antennaStatus->setTheme();
    m_attenuatorPanel->setTheme();
    auto *btn = findChild<QPushButton*>("themeBtn");
    if (btn) btn->setText(m_isDark ? "☀" : "☾");
    m_statusLabel->style()->unpolish(m_statusLabel);
    m_statusLabel->style()->polish(m_statusLabel);
    QString dataDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/rf_frontend_control";
    QSettings settings(dataDir + "/settings.ini", QSettings::IniFormat);
    settings.setValue("theme/dark", m_isDark);
}
