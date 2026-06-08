#include "cdccomm.h"
#include <QDebug>

CdccComm::CdccComm(QObject *parent)
    : QObject(parent), m_serial(new QSerialPort(this)), m_currentPort("")
{
    connect(m_serial, &QSerialPort::readyRead, this, &CdccComm::onReadyRead);
    connect(m_serial, &QSerialPort::errorOccurred, this, &CdccComm::onErrorOccurred);
}

CdccComm::~CdccComm() {
    close();
}

QStringList CdccComm::availablePorts() {
    QStringList ports;
    for (const auto &info : QSerialPortInfo::availablePorts()) {
        ports.append(info.portName());
    }
    return ports;
}

bool CdccComm::open(const QString &portName, int baud) {
    if (m_serial->isOpen()) m_serial->close();

    m_serial->setPortName(portName);
    m_serial->setBaudRate(baud);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serial->open(QIODevice::ReadWrite)) {
        emit errorOccurred(tr("无法打开 %1: %2").arg(portName, m_serial->errorString()));
        return false;
    }

    m_currentPort = portName;
    emit connected();
    qDebug() << "CDC connected:" << portName;
    return true;
}

void CdccComm::close() {
    if (m_serial->isOpen()) {
        m_serial->close();
        m_currentPort = "";
        emit disconnected();
    }
}

bool CdccComm::isOpen() const {
    return m_serial->isOpen();
}

bool CdccComm::sendTemplate(const std::array<uint8_t, 12> &data) {
    if (!m_serial->isOpen()) {
        emit errorOccurred(tr("串口未打开"));
        return false;
    }

    QByteArray packet(reinterpret_cast<const char*>(data.data()), 12);
    qint64 written = m_serial->write(packet);
    if (written != 12) {
        emit errorOccurred(tr("发送失败: 写了 %1 字节, 期望 12").arg(written));
        return false;
    }
    m_serial->flush();
    return true;
}

QString CdccComm::sendAndReceive(const std::array<uint8_t, 12> &data, int timeoutMs) {
    if (!sendTemplate(data)) return {};

    // Wait for echo/response from MCU
    if (!m_serial->waitForBytesWritten(50)) {}
    if (!m_serial->waitForReadyRead(timeoutMs)) {
        return tr("(无回复)");
    }

    QByteArray resp = m_serial->readAll();
    return QString::fromUtf8(resp);
}

void CdccComm::onReadyRead() {
    QByteArray data = m_serial->readAll();
    m_rxBuffer.append(data);
    emit dataReceived(data);
}

void CdccComm::onErrorOccurred(QSerialPort::SerialPortError error) {
    if (error == QSerialPort::NoError) return;
    emit errorOccurred(m_serial->errorString());
    if (error == QSerialPort::ResourceError) {
        close();
    }
}
