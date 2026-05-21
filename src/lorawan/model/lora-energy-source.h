/*
 * Adapted from basic-energy-source.h
 */

#ifndef LORA_ENERGY_SOURCE_H
#define LORA_ENERGY_SOURCE_H

#include "ns3/energy-source.h"

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/traced-value.h"

namespace ns3
{
namespace lorawan
{

/**
 * @ingroup lorawan
 * @brief A simple energy model that supports consuming a fixed amount of energy,
 * while preserving the BasicEnergySource functionality.
 *
 */
class LoraEnergySource : public energy::EnergySource
{
  public:
    /**
     * @brief Get the type ID.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();
    LoraEnergySource();
    ~LoraEnergySource() override;

    /**
     * @brief Consume a certain amount of energy.
     * @param energyToConsumeJ The energy to decrease.
     */
    void ConsumeFixedEnergy (double energyToConsumeJ);

    /**
     * @return Initial energy stored in energy source, in Joules.
     *
     * Implements GetInitialEnergy.
     */
    double GetInitialEnergy() const override;

    /**
     * @returns Supply voltage at the energy source.
     *
     * Implements GetSupplyVoltage.
     */
    double GetSupplyVoltage() const override;

    /**
     * @return Remaining energy in energy source, in Joules
     *
     * Implements GetRemainingEnergy.
     */
    double GetRemainingEnergy() override;

    /**
     * @returns Energy fraction.
     *
     * Implements GetEnergyFraction.
     */
    double GetEnergyFraction() override;

    /**
     * Implements UpdateEnergySource.
     */
    void UpdateEnergySource() override;

    /**
     * @param initialEnergyJ Initial energy, in Joules
     *
     * Sets initial energy stored in the energy source. Note that initial energy
     * is assumed to be set before simulation starts and is set only once per
     * simulation.
     */
    void SetInitialEnergy(double initialEnergyJ);

    /**
     * @param supplyVoltageV Supply voltage at the energy source, in Volts.
     *
     * Sets supply voltage of the energy source.
     */
    void SetSupplyVoltage(double supplyVoltageV);

    /**
     * @param interval Energy update interval.
     *
     * This function sets the interval between each energy update.
     */
    void SetEnergyUpdateInterval(Time interval);

    /**
     * @returns The interval between each energy update.
     */
    Time GetEnergyUpdateInterval() const;

  private:
    /// Defined in ns3::Object
    void DoInitialize() override;

    /// Defined in ns3::Object
    void DoDispose() override;

    /**
     * Handles the remaining energy going to zero event. This function notifies
     * all the energy models aggregated to the node about the energy being
     * depleted. Each energy model is then responsible for its own handler.
     */
    void HandleEnergyDrainedEvent();

    /**
     * Handles the remaining energy exceeding the high threshold after it went
     * below the low threshold. This function notifies all the energy models
     * aggregated to the node about the energy being recharged. Each energy model
     * is then responsible for its own handler.
     */
    void HandleEnergyRechargedEvent();

    /**
     * Calculates remaining energy. This function uses the total current from all
     * device models to calculate the amount of energy to decrease. The energy to
     * decrease is given by:
     *    energy to decrease = total current * supply voltage * time duration
     * This function subtracts the calculated energy to decrease from remaining
     * energy.
     */
    void CalculateRemainingEnergy();

  private:
    double m_initialEnergyJ; //!< initial energy, in Joules
    double m_supplyVoltageV; //!< supply voltage, in Volts
    double m_lowBatteryTh;   //!< low battery threshold, as a fraction of the initial energy
    double m_highBatteryTh;  //!< high battery threshold, as a fraction of the initial energy
    /**
     * set to true when the remaining energy goes below the low threshold,
     * set to false again when the remaining energy exceeds the high threshold
     */
    bool m_depleted;
    TracedValue<double> m_remainingEnergyJ; //!< remaining energy, in Joules
    EventId m_energyUpdateEvent;            //!< energy update event
    Time m_lastUpdateTime;                  //!< last update time
    Time m_energyUpdateInterval;            //!< energy update interval
};

} // namespace energy
} // namespace ns3

#endif /* BASIC_ENERGY_SOURCE_H */
