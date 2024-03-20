#pragma once

void MavlinkInitialize();
void SendHeartbeatTask(void * parameters);
void DecodeTelemetryTask(void * parameters);