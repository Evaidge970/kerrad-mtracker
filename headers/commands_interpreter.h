#pragma once

void InitHLBuffer();
void InterpretCommand(uint16_t *inputBuffer, uint16_t *outputBuffer);
void InitializeFrameHead(uint16_t n1, uint16_t n2, uint16_t * buf);
