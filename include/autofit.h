#pragma once
#include "includes.h"
#include <QtPlugin>

class AutofitImpl;
class ccPointCloud;

class Autofit : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
    // Q_PLUGIN_METADATA(IID "nl.Bocari.cc.plugin.Autofit" FILE "../info.json")
    Q_PLUGIN_METADATA(IID "cc.plugins.stdplugin" FILE "../info.json")

public:
    explicit Autofit(QObject* parent = nullptr);
    ~Autofit() override;

    // Verplichte ccStdPluginInterface methodes
    QList<QAction*> getActions() override { return {m_action}; }
    ContactList     getAuthors() const override { return {{"Richard Rombouts", "rf.rombouts@gmail.com"}}; }
    QString         getDescription() const override { return "Plugin voor vinden van pijpen, bochten, ..."; }
    QIcon           getIcon() const override { return m_action->icon(); } // RR!!!
    ContactList     getMaintainers() const override { return getAuthors(); }
    QString         getName() const override { return "Autofit"; }
    ReferenceList   getReferences() const override { return ReferenceList(); }
    CC_PLUGIN_TYPE  getType() const override { return CC_STD_PLUGIN; }
    bool            isCore() const override { return false; }
    void            onNewSelection(const ccHObject::Container& selectedEntities) override;
    bool            start() override { return true; }
    void            stop() override {}

private:
    QAction*                     m_action; // Behoudt actie in basisklasse voor signal/slot connecties
    ccPointCloud*                m_selectedCloud;
    std::unique_ptr<AutofitImpl> m_pimpl;
private slots:
    virtual void performFit();
};
