#ifndef TEMPLATEMANAGER_H
#define TEMPLATEMANAGER_H

#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <vector>
#include "controlword.h"

struct TemplateEntry {
    QString name;
    QString description;
    ControlWord word;
    bool builtIn = false; // 系统内置模板，不可覆盖/删除
};

class TemplateManager {
public:
    TemplateManager();

    // 从文件加载模板库
    bool loadFromFile(const QString &filePath);
    bool saveToFile(const QString &filePath) const;

    // 从 ControlWord 创建模板
    void addTemplate(const QString &name, const QString &desc, const ControlWord &word);
    void addBuiltinTemplate(const QString &name, const QString &desc, const ControlWord &word);
    void removeTemplate(int index);
    void clear();

    // 查询
    int count() const { return m_templates.size(); }
    const TemplateEntry &at(int index) const { return m_templates.at(index); }
    TemplateEntry &at(int index) { return m_templates.at(index); }

    // 查找
    int findByName(const QString &name) const;

    // 内置默认模板
    void loadDefaults();

private:
    std::vector<TemplateEntry> m_templates;
};

#endif // TEMPLATEMANAGER_H
