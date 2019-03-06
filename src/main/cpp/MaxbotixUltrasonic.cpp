/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MaxbotixUltrasonic.h"

MaxbotixUltrasonic::MaxbotixUltrasonic(int channel)
{
    m_channel = new frc::AnalogInput(channel);
}

double MaxbotixUltrasonic::GetVoltage() const
{
    return m_channel->GetVoltage();
}
