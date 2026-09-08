/*
 * ClockGenerator.c
 *
 * This file contains the driver for the Cypress CY22393 PLL clock generator.
 * 20MHz HC-49US crystal used, 30 ohms ESR, 7 pF shunt capacitance.
 *
 * Updated to disable the PLL before update. This is recomended in the data sheet to
 * prevent any out of bound PLL conditions.
 *
 *  Created: 12/27/2011 7:26:02 PM
 *  Author: Gordon Anderson
 */

#include <Arduino.h> 
#include "stdlib.h"
#include "ClockGenerator.h"
#include "RFdriver.h"
#include <SoftwareI2C.h>

CY22393_regs CYregs;

// PLL parameters
int16_t  P0;
int16_t  P;
int16_t  Q;
int16_t  Div;
int16_t  LF;

int Ref = 8000000;

void SetRef(int Freq)
{
  Ref = Freq;
}

// Searches for the divider (Div) and PLL feedback (P, Q) values that make the
// CY22393's PLL output ClockOut Hz from the Ref Hz reference (set with
// SetRef()), minimizing the frequency error. Also derives the loop filter
// setting LF from the resulting P value, per the CY22393 datasheet. Leaves
// its result in the P0/P/Q/Div/LF globals for SetPLL2freq()/SetPLL3freq() to
// consume, and returns the achieved frequency error in Hz.
int FindPQ(int ClockOut)
{
  static int  Pt, Qt, Divider,D, MaxQ, AClockOut;
  static int  Cerror, CerrorMin, A, B;

  MaxQ = 200;
  // Determine starting divider
  Divider = ((Ref / MaxQ) * 1600) / ClockOut;
  if (Divider > 117) Divider = 117;
  if (Divider < 11) Divider = 11; 
  // Loop through 20 Divider values
  CerrorMin = ClockOut;
  for (D = Divider - 10; D <= Divider + 10; D++)
  {
    // Loop through Qt values
    A = (ClockOut / 1000) * D;
    B = Ref / 1000;
    for (Qt = 2; Qt <= MaxQ; Qt++)
    {
      // Calculate Pt
      Pt = (A * Qt) / B;
      if(Pt > 1600) Pt = 1600;
      // Now calculate Actual ClockOut
      AClockOut = ((Ref / Qt) * Pt) / D;
      Cerror = labs(AClockOut - ClockOut);
      if (Cerror < CerrorMin)
      {
        CerrorMin = Cerror;
        Q = Qt;
        P = Pt;
        Div = D;
        if (Cerror < 50) Cerror = 0;
      }
      if (Cerror == 0) break;
    }
    if (Cerror == 0) break;
  }
  P0 = P & 1;
  P = ((P - P0) / 2) - 3;
  Q = Q - 2;
  LF = 0;
  if (P > 231) LF = 1;
  if (P > 626) LF = 2;
  if (P > 834) LF = 3;
  if (P > 1043) LF = 4;
  return (CerrorMin);
}

// Initializes the CY22393's register image (clock A/C on PLL2, clock B/D on
// PLL3, crystal oscillator config) and writes the whole CY22393_regs block to
// the device at TWI address adr over the software I2C bus (WireS1). Both
// PLLs are left disabled (Misc bit 6 clear) at this point; SetPLL2freq()/
// SetPLL3freq() enable them once a frequency has been set. Returns 0 on
// success or the TWI error code from Wire::endTransmission() on failure.
int CY_Init(int8_t adr)
{
  int iStat;

  //
  // Initialize the data structure
  //
  CYregs.RegAdd = 0x08;
  CYregs.Osc = 0b10010101;		// Crystal, 7PF, 30 ohms
  // Set clock A and C to PLL2
  //   Clk*_fs[2:0] = 100
  CYregs.ClkA_Div_DS0 = 0b00000000;
  CYregs.ClkA_Div_DS1 = 0b00000000;
  CYregs.ClkC_Div = 0b00000000;
  // Set clock B and D to PLL3
  //   Clk*_fs[2:0] = 110
  CYregs.ClkB_Div_DS0 = 0b00000000;
  CYregs.ClkB_Div_DS1 = 0b00000000;
  CYregs.ClkD_Div = 0b00000000;
  CYregs.ClkABCD_FS = 0b11101110;	// FS[2:1]
  // Set output duty cycle, enables, clkE_div
  CYregs.Clk_ACadj = 0b01010101;
  // Set the output drive level
  CYregs.Clk_DCadj = 0b01010101;
  // PLL2
  CYregs.PLL2_Q = 0;
  CYregs.PLL2_P = 0;
  CYregs.PLL2_Misc = 0b01000000;
  // PLL3
  CYregs.PLL3_Q = 0;
  CYregs.PLL3_P = 0;
  CYregs.PLL3_Misc = 0b01000000;
  //
  // Send to the device using TWI
  //
  WireS1.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;
  for (int i = 0; i < (int)sizeof(CY22393_regs); i++)
  {
    WireS1.write(bvals[i]);
  }
  if ((iStat = WireS1.endTransmission()) != 0) return (iStat);
  return (0);
}

// Sets PLL2 (clocks A and C, i.e. RF channel 1's frequency source, see
// CY_Init()) to Freq Hz: disables the PLL, recomputes and writes the full
// register block via FindPQ(), then re-enables it. Disabling around the
// update is per the CY22393 datasheet recommendation, to avoid a transient
// out-of-lock condition while the divider/PLL values change. Returns 0 on
// success or the TWI error code from Wire::endTransmission() on failure.
int SetPLL2freq(int8_t adr, int Freq)
{
  int iStat;

  //
  // Setup PLL2 parameters
  //
  FindPQ(Freq);			// Determine the PLL values
  CYregs.PLL2_P = P & 0xFF;
  CYregs.PLL2_Misc = (CYregs.PLL2_Misc & 0xC0) | (LF << 3) | (P0 << 2) | (P >> 8);
  CYregs.PLL2_Q = Q;
  CYregs.ClkA_Div_DS0 = (CYregs.ClkA_Div_DS0 & 0x80) | (Div & 0x7F);
  CYregs.ClkA_Div_DS1 = (CYregs.ClkA_Div_DS1 & 0x80) | (Div & 0x7F);
  CYregs.ClkC_Div = (CYregs.ClkC_Div & 0x80) | (Div & 0x7F);
  // Disable PLL2
  CYregs.PLL2_Misc &= ~0x40;
  WireS1.beginTransmission(adr);
  WireS1.write(0x13);
  WireS1.write(CYregs.PLL2_Misc);
  WireS1.endTransmission();
  //
  // Send data structure to the device using TWI
  //
  WireS1.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;

  for (int i = 0; i < (int)sizeof(CY22393_regs); i++)
  {
    WireS1.write(bvals[i]);
  }
  if ((iStat = WireS1.endTransmission()) != 0) return (iStat);
  // Enable PLL2
  CYregs.PLL2_Misc |= 0x40;
  WireS1.beginTransmission(adr);
  WireS1.write(0x13);
  WireS1.write(CYregs.PLL2_Misc);
  WireS1.endTransmission();
  return (0);
}

// Sets PLL3 (clocks B and D, i.e. RF channel 2's frequency source, see
// CY_Init()) to Freq Hz. See SetPLL2freq() above for the procedure and
// return value; this is the same thing applied to PLL3's registers.
int SetPLL3freq(int8_t adr, int Freq)
{
  int iStat;

  //
  // Setup PLL3 parameters
  //
  FindPQ(Freq);			// Determine the PLL values
  CYregs.PLL3_P = P & 0xFF;
  CYregs.PLL3_Misc = (CYregs.PLL3_Misc & 0xC0) | (LF << 3) | (P0 << 2) | (P >> 8);
  CYregs.PLL3_Q = Q;
  CYregs.ClkB_Div_DS0 = (CYregs.ClkB_Div_DS0 & 0x80) | (Div & 0x7F);
  CYregs.ClkB_Div_DS1 = (CYregs.ClkB_Div_DS1 & 0x80) | (Div & 0x7F);
  CYregs.ClkD_Div = (CYregs.ClkD_Div & 0x80) | (Div & 0x7F);
  // Disable PLL3
  CYregs.PLL3_Misc &= ~0x40;
  WireS1.beginTransmission(adr);
  WireS1.write(0x16);
  WireS1.write(CYregs.PLL3_Misc);
  WireS1.endTransmission();
  //
  // Send to the device using TWI
  //
  WireS1.beginTransmission(adr);
  byte *bvals = (byte *)&CYregs;
  for (int i = 0; i < (int)sizeof(CY22393_regs); i++)
  {
    WireS1.write(bvals[i]);
  }
  if ((iStat = WireS1.endTransmission()) != 0) return (iStat);
  // Enable PLL3
  CYregs.PLL3_Misc |= 0x40;
  WireS1.beginTransmission(adr);
  WireS1.write(0x16);
  WireS1.write(CYregs.PLL3_Misc);
  WireS1.endTransmission();
  return (0);
}
