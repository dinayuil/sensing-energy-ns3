/*
 * Adapted from basic-energy-source-helper.h
 */

#ifndef LORA_ENERGY_SOURCE_HELPER_H
#define LORA_ENERGY_SOURCE_HELPER_H

#include "ns3/energy-model-helper.h"
#include "ns3/node.h"

namespace ns3
{

/**
 * @ingroup energy
 * @brief Creates a LoraEnergySource object.
 *
 */
class LoraEnergySourceHelper : public EnergySourceHelper
{
  public:
    LoraEnergySourceHelper();
    ~LoraEnergySourceHelper() override;
    void Set(std::string name, const AttributeValue& v) override;

  private:
    Ptr<energy::EnergySource> DoInstall(Ptr<Node> node) const override;

  private:
    ObjectFactory m_loraEnergySource; //!< Energy source factory
};

} // namespace ns3

#endif /* LORA_ENERGY_SOURCE_HELPER_H */
