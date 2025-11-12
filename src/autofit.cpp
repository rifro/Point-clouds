#include "autofit.h"
#include "autofit_impl.h"
#include "includes.h"
#include "strict.h"

#include "voxelprocessor.h"
// using CCCoreLib::DegreesToRadians;

Autofit::Autofit(QObject* parent)
    : QObject(parent), m_action(nullptr), m_selectedCloud(nullptr), m_pimpl(new AutofitImpl)
{
    if(!m_action)
    {
        m_action = new QAction("Perform autofit", this);
        m_action->setIcon(QIcon(":/icons/bochtje50.png"));
        connect(m_action, &QAction::triggered, this, &Autofit::performFit);
    }
    m_action->setEnabled(false);
}

Autofit::~Autofit() = default;

void Autofit::onNewSelection(const ccHObject::Container& selectedEntities)
{
    bool enabled = selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD);
    m_action->setEnabled(enabled);
    m_selectedCloud = enabled ? static_cast<ccPointCloud*>(selectedEntities[0]) : nullptr;
}

void Autofit::performFit()
{
    if(m_pimpl && m_selectedCloud != nullptr) m_pimpl->performFit(m_app, m_selectedCloud);
}
