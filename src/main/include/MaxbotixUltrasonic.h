/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Counter.h>
#include <frc/ErrorBase.h>
#include <frc/PIDSource.h>
#include <frc/AnalogInput.h>

class MaxbotixUltrasonic : public frc::ErrorBase, public frc::SendableBase, public frc::PIDSource {
 public:
  MaxbotixUltrasonic(int channel);
/*
  MaxbotixUltrasonic(int channel,
                     bool use_units,
                     double min_voltage,
                     double max_voltage,
                     double min_distance,
                     double max_distance);
*/
  double GetVoltage() const;
  private:
    frc::AnalogInput *m_channel;
};
