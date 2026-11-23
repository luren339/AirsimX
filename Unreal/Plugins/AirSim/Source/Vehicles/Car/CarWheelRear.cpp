// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelRear.h"

UCarWheelRear::UCarWheelRear()
{
    // Chaos vehicle wheel setup; property names differ from the UE4 PhysX wheel.
    WheelRadius = 18.f;
    WheelWidth = 15.0f;
    WheelMass = 20.0f;
    bAffectedByHandbrake = true;
    MaxSteerAngle = 0.f;

    SuspensionForceOffset = FVector::ZeroVector;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SpringRate = 2500.0f;
    SuspensionDampingRatio = 1.05f;
}
