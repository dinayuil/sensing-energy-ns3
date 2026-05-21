/*
 * Adapted from basic-energy-source-helper.cc
 */

#include "lora-energy-source-helper.h"

#include "ns3/lora-energy-source.h"

namespace ns3
{

LoraEnergySourceHelper::LoraEnergySourceHelper()
{
    m_loraEnergySource.SetTypeId("ns3::LoraEnergySource");
}

LoraEnergySourceHelper::~LoraEnergySourceHelper()
{
}

void
LoraEnergySourceHelper::Set(std::string name, const AttributeValue& v)
{
    m_loraEnergySource.Set(name, v);
}

Ptr<energy::EnergySource>
LoraEnergySourceHelper::DoInstall(Ptr<Node> node) const
{
    NS_ASSERT(node);
    Ptr<energy::EnergySource> source = m_loraEnergySource.Create<energy::EnergySource>();
    NS_ASSERT(source);
    source->SetNode(node);
    return source;
}

} // namespace ns3
