#include "templatemanager.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

TemplateManager::TemplateManager() {}

bool TemplateManager::loadFromFile(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open template file:" << filePath;
        return false;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    if (!doc.isObject()) return false;

    QJsonObject root = doc.object();
    QJsonArray arr = root["templates"].toArray();

    m_templates.clear();
    for (const auto &val : arr) {
        QJsonObject obj = val.toObject();
        TemplateEntry entry;
        entry.name = obj["name"].toString();
        entry.description = obj["description"].toString();

        std::array<uint8_t, 12> bytes{};
        QJsonArray byteArr = obj["bytes"].toArray();
        for (int i = 0; i < 12 && i < byteArr.size(); i++) {
            bytes[i] = static_cast<uint8_t>(byteArr[i].toInt(0) & 0xFF);
        }

        if (obj.contains("dirtyMask")) {
            std::array<uint8_t, 12> dirty{};
            QJsonArray dirtyArr = obj["dirtyMask"].toArray();
            for (int i = 0; i < 12 && i < dirtyArr.size(); i++)
                dirty[i] = static_cast<uint8_t>(dirtyArr[i].toInt(0) & 0xFF);
            entry.word.setRawBytes(bytes, dirty);
        } else {
            entry.word.setRawBytes(bytes); // 旧数据 → 全脏
        }
        m_templates.push_back(entry);
    }

    qDebug() << "Loaded" << m_templates.size() << "templates from" << filePath;
    return true;
}

bool TemplateManager::saveToFile(const QString &filePath) const {
    QJsonArray arr;
    for (const auto &t : m_templates) {
        QJsonObject obj;
        obj["name"] = t.name;
        obj["description"] = t.description;

        QJsonArray byteArr;
        auto bytes = t.word.rawBytes();
        for (auto b : bytes) byteArr.append(b);
        obj["bytes"] = byteArr;

        // 脏掩码：全 FF 时省略以节省空间
        auto dirty = t.word.dirtyBytes();
        bool allFF = true;
        for (auto b : dirty) { if (b != 0xFF) { allFF = false; break; } }
        if (!allFF) {
            QJsonArray dirtyArr;
            for (auto b : dirty) dirtyArr.append(b);
            obj["dirtyMask"] = dirtyArr;
        }

        arr.append(obj);
    }

    QJsonObject root;
    root["version"] = 1;
    root["templates"] = arr;

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot write template file:" << filePath;
        return false;
    }

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

void TemplateManager::addTemplate(const QString &name, const QString &desc, const ControlWord &word) {
    TemplateEntry entry;
    entry.name = name;
    entry.description = desc;
    entry.word = word;
    m_templates.push_back(entry);
}

void TemplateManager::addBuiltinTemplate(const QString &name, const QString &desc, const ControlWord &word) {
    TemplateEntry entry;
    entry.name = name;
    entry.description = desc;
    entry.word = word;
    entry.builtIn = true;
    m_templates.push_back(entry);
}

void TemplateManager::removeTemplate(int index) {
    if (index >= 0 && index < static_cast<int>(m_templates.size()))
        m_templates.erase(m_templates.begin() + index);
}

void TemplateManager::clear() {
    m_templates.clear();
}

int TemplateManager::findByName(const QString &name) const {
    for (size_t i = 0; i < m_templates.size(); i++) {
        if (m_templates[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

void TemplateManager::loadDefaults() {
    m_templates.clear();

    auto addDef = [&](const QString &name, const QString &desc, ControlWord &cw) {
        addBuiltinTemplate(name, desc, cw);
    };

    ControlWord cw;
    for (int m = 1; m <= 3; m++)
        for (int c = 1; c <= 3; c++) {
            cw.setIFBand(m, c, false);
            cw.setIFAttenuation(m, c, 15);
        }
    cw.markAllDirty();
    addDef("STANDBY", "天线全关，IF 衰减最大 + 窄带", cw);
    qDebug() << "Default templates loaded:" << m_templates.size();
}
