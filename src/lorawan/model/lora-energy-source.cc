/*
 * Adapted from basic-energy-source.cc
 */

#include "ns3/lora-energy-source.h"

#include "ns3/assert.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("LoraEnergySource");
NS_OBJECT_ENSURE_REGISTERED(LoraEnergySource);

TypeId
LoraEnergySource::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LoraEnergySource")
            .SetParent<EnergySource>()
            .SetGroupName("Energy")
            .AddConstructor<LoraEnergySource>()
            .AddAttribute("LoraEnergySourceInitialEnergyJ",
                          "Initial energy stored in energy source.",
                          DoubleValue(10), // in Joules
                          MakeDoubleAccessor(&LoraEnergySource::SetInitialEnergy,
                                             &LoraEnergySource::GetInitialEnergy),
                          MakeDoubleChecker<double>())
            .AddAttribute("LoraEnergySupplyVoltageV",
                          "Initial supply voltage for energy source.",
                          DoubleValue(3.0), // in Volts
                          MakeDoubleAccessor(&LoraEnergySource::SetSupplyVoltage,
                                             &LoraEnergySource::GetSupplyVoltage),
                          MakeDoubleChecker<double>())
            .AddAttribute("LoraEnergyLowBatteryThreshold",
                          "Low battery threshold for energy source.",
                          DoubleValue(0.10), // as a fraction of the initial energy
                          MakeDoubleAccessor(&LoraEnergySource::m_lowBatteryTh),
                          MakeDoubleChecker<double>())
            .AddAttribute("LoraEnergyHighBatteryThreshold",
                          "High battery threshold for energy source.",
                          DoubleValue(0.15), // as a fraction of the initial energy
                          MakeDoubleAccessor(&LoraEnergySource::m_highBatteryTh),
                          MakeDoubleChecker<double>())
            .AddAttribute("PeriodicEnergyUpdateInterval",
                          "Time between two consecutive periodic energy updates.",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&LoraEnergySource::SetEnergyUpdateInterval,
                                           &LoraEnergySource::GetEnergyUpdateInterval),
                          MakeTimeChecker())
            .AddTraceSource("RemainingEnergy",
                            "Remaining energy at LoraEnergySource.",
                            MakeTraceSourceAccessor(&LoraEnergySource::m_remainingEnergyJ),
                            "ns3::TracedValueCallback::Double");
    return tid;
}

LoraEnergySource::LoraEnergySource()
{
    NS_LOG_FUNCTION(this);
    m_lastUpdateTime = Seconds(0);
    m_depleted = false;
}

LoraEnergySource::~LoraEnergySource()
{
    NS_LOG_FUNCTION(this);
}

void
LoraEnergySource::SetInitialEnergy(double initialEnergyJ)
{
    NS_LOG_FUNCTION(this << initialEnergyJ);
    NS_ASSERT(initialEnergyJ >= 0);
    m_initialEnergyJ = initialEnergyJ;
    m_remainingEnergyJ = m_initialEnergyJ;
}

void
LoraEnergySource::SetSupplyVoltage(double supplyVoltageV)
{
    NS_LOG_FUNCTION(this << supplyVoltageV);
    m_supplyVoltageV = supplyVoltageV;
}

void
LoraEnergySource::SetEnergyUpdateInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_energyUpdateInterval = interval;
}

Time
LoraEnergySource::GetEnergyUpdateInterval() const
{
    NS_LOG_FUNCTION(this);
    return m_energyUpdateInterval;
}

double
LoraEnergySource::GetSupplyVoltage() const
{
    NS_LOG_FUNCTION(this);
    return m_supplyVoltageV;
}

double
LoraEnergySource::GetInitialEnergy() const
{
    NS_LOG_FUNCTION(this);
    return m_initialEnergyJ;
}

double
LoraEnergySource::GetRemainingEnergy()
{
    NS_LOG_FUNCTION(this);
    // update energy source to get the latest remaining energy.
    UpdateEnergySource();
    return m_remainingEnergyJ;
}

double
LoraEnergySource::GetEnergyFraction()
{
    NS_LOG_FUNCTION(this);
    // update energy source to get the latest remaining energy.
    UpdateEnergySource();
    return m_remainingEnergyJ / m_initialEnergyJ;
}

void
LoraEnergySource::UpdateEnergySource()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("LoraEnergySource:Updating remaining energy.");

    double remainingEnergy = m_remainingEnergyJ;
    CalculateRemainingEnergy();

    m_lastUpdateTime = Simulator::Now();

    if (!m_depleted && m_remainingEnergyJ <= m_lowBatteryTh * m_initialEnergyJ)
    {
        m_depleted = true;
        HandleEnergyDrainedEvent();
    }
    else if (m_depleted && m_remainingEnergyJ > m_highBatteryTh * m_initialEnergyJ)
    {
        m_depleted = false;
        HandleEnergyRechargedEvent();
    }
    else if (m_remainingEnergyJ != remainingEnergy)
    {
        NotifyEnergyChanged();
    }

    if (m_energyUpdateEvent.IsExpired())
    {
        m_energyUpdateEvent = Simulator::Schedule(m_energyUpdateInterval,
                                                  &LoraEnergySource::UpdateEnergySource,
                                                  this);
    }
}

/*
 * Private functions start here.
 */

void
LoraEnergySource::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    UpdateEnergySource(); // start periodic update
}

void
LoraEnergySource::DoDispose()
{
    NS_LOG_FUNCTION(this);
    BreakDeviceEnergyModelRefCycle(); // break reference cycle
}

void
LoraEnergySource::HandleEnergyDrainedEvent()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("LoraEnergySource:Energy depleted!");
    NotifyEnergyDrained(); // notify DeviceEnergyModel objects
}

void
LoraEnergySource::HandleEnergyRechargedEvent()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("LoraEnergySource:Energy recharged!");
    NotifyEnergyRecharged(); // notify DeviceEnergyModel objects
}

void
LoraEnergySource::CalculateRemainingEnergy()
{
    NS_LOG_FUNCTION(this);
    double totalCurrentA = CalculateTotalCurrent();
    Time duration = Simulator::Now() - m_lastUpdateTime;
    NS_ASSERT(duration.IsPositive());
    // energy = current * voltage * time
    double energyToDecreaseJ = (totalCurrentA * m_supplyVoltageV * duration).GetSeconds();
    NS_ASSERT(m_remainingEnergyJ >= energyToDecreaseJ);
    m_remainingEnergyJ -= energyToDecreaseJ;
    NS_LOG_DEBUG("LoraEnergySource:Remaining energy = " << m_remainingEnergyJ);
}

void
LoraEnergySource::ConsumeFixedEnergy(double energyToConsumeJ)
{
    NS_LOG_FUNCTION(this << energyToConsumeJ);

    double previousEnergy = m_remainingEnergyJ;
    m_remainingEnergyJ -= energyToConsumeJ;
    // avoid negative energy
    if (m_remainingEnergyJ <= 0)
    {
        m_remainingEnergyJ = 0;
    }

    NS_LOG_DEBUG("LoraEnergySource: Fixed drop " << energyToConsumeJ
                                                 << "J. Remaining: " << m_remainingEnergyJ);

    if (!m_depleted && m_remainingEnergyJ <= m_lowBatteryTh * m_initialEnergyJ)
    {
        m_depleted = true;
        HandleEnergyDrainedEvent();
    }
    else if (m_remainingEnergyJ != previousEnergy)
    {
        NotifyEnergyChanged();
    }
}

} // namespace lorawan
} // namespace ns3
