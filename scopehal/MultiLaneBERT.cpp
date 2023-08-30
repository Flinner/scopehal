/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "MultiLaneBERT.h"
#include "EyeWaveform.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

MultiLaneBERT::MultiLaneBERT(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
{
	//Set a very long socket timeout because initial connection creation takes forever
	auto socktrans = dynamic_cast<SCPISocketTransport*>(transport);
	unsigned int timeoutSec = 30;
	unsigned int timeoutUs = timeoutSec * 1000 * 1000;
	if(socktrans)
		socktrans->SetTimeouts(timeoutUs, timeoutUs);

	//Don't push changes to hardware every time we poke a single channel setting
	transport->SendCommandQueued("DEFER");

	//Change the data rate
	SetDataRate(10312500000LL);

	//Add and provide default configuration for pattern generator channels
	int nchans = 4;
	m_rxChannelBase = nchans;
	for(int i=0; i<nchans; i++)
	{
		m_channels.push_back(new BERTOutputChannel(string("TX") + to_string(i+1), this, "#808080", i));
		SetTxPattern(i, PATTERN_PRBS7);
		SetTxInvert(i, false);
		SetTxDriveStrength(i, 0.2);
		SetTxEnable(i, true);
		SetTxPreCursor(i, 0);
		SetTxPostCursor(i, 0);
	}

	//Add pattern checker channels
	for(int i=0; i<nchans; i++)
	{
		m_channels.push_back(new BERTInputChannel(string("RX") + to_string(i+1), this, "#808080", i+nchans));
		SetRxPattern(i+nchans, PATTERN_PRBS7);
		SetRxInvert(i+nchans, false);
	}

	//Apply the deferred changes
	//This results in a single API call instead of four for each channel, causing a massive speedup during initialization
	transport->SendCommandQueued("APPLY");

	//Set up default custom pattern
	SetGlobalCustomPattern(0xff00);

	//Set the output mux refclk to LO/32
	SetRefclkOutMux(LO_DIV32_OR_80);
}

MultiLaneBERT::~MultiLaneBERT()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Instrument config

string MultiLaneBERT::GetDriverNameInternal()
{
	return "mlbert";
}

uint32_t MultiLaneBERT::GetInstrumentTypesForChannel(size_t /*i*/)
{
	return INST_BERT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RX pattern checker control

BERT::Pattern MultiLaneBERT::GetRxPattern(size_t i)
{
	return m_rxPattern[i - m_rxChannelBase];
}

void MultiLaneBERT::SetRxPattern(size_t i, Pattern pattern)
{
	switch(pattern)
	{
		case PATTERN_PRBS7:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS7");
			break;
		case PATTERN_PRBS9:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS9");
			break;
		case PATTERN_PRBS15:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS15");
			break;
		case PATTERN_PRBS23:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS23");
			break;
		case PATTERN_PRBS31:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS31");
			break;

		case PATTERN_CUSTOM:
		default:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY AUTO");
	}

	m_rxPattern[i - m_rxChannelBase] = pattern;
}

vector<BERT::Pattern> MultiLaneBERT::GetAvailableRxPatterns(size_t /*i*/)
{
	vector<Pattern> ret;
	ret.push_back(PATTERN_PRBS7);
	ret.push_back(PATTERN_PRBS9);
	ret.push_back(PATTERN_PRBS15);
	ret.push_back(PATTERN_PRBS23);
	ret.push_back(PATTERN_PRBS31);
	ret.push_back(PATTERN_AUTO);
	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RX input buffer control

bool MultiLaneBERT::GetRxInvert(size_t i)
{
	return m_rxInvert[i - m_rxChannelBase];
}

void MultiLaneBERT::SetRxInvert(size_t i, bool invert)
{
	if(invert)
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":INVERT 1");
	else
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":INVERT 0");

	m_rxInvert[i - m_rxChannelBase] = invert;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TX pattern generator control

vector<BERT::Pattern> MultiLaneBERT::GetAvailableTxPatterns(size_t /*i*/)
{
	vector<Pattern> ret;
	ret.push_back(PATTERN_PRBS7);
	ret.push_back(PATTERN_PRBS9);
	ret.push_back(PATTERN_PRBS15);
	ret.push_back(PATTERN_PRBS23);
	ret.push_back(PATTERN_PRBS31);
	ret.push_back(PATTERN_CUSTOM);
	return ret;
}

BERT::Pattern MultiLaneBERT::GetTxPattern(size_t i)
{
	return m_txPattern[i];
}

void MultiLaneBERT::SetTxPattern(size_t i, Pattern pattern)
{
	switch(pattern)
	{
		case PATTERN_PRBS7:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS7");
			break;
		case PATTERN_PRBS9:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS9");
			break;
		case PATTERN_PRBS15:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS15");
			break;
		case PATTERN_PRBS23:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS23");
			break;
		case PATTERN_PRBS31:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY PRBS31");
			break;

		case PATTERN_CUSTOM:
		default:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POLY USER");
	}

	m_txPattern[i] = pattern;
}

bool MultiLaneBERT::IsCustomPatternPerChannel()
{
	return false;
}

size_t MultiLaneBERT::GetCustomPatternLength()
{
	return 16;
}

void MultiLaneBERT::SetGlobalCustomPattern(uint64_t pattern)
{
	m_transport->SendCommandQueued(string("USERPATTERN ") + to_string_hex(pattern));

	m_txCustomPattern = pattern;
}

uint64_t MultiLaneBERT::GetGlobalCustomPattern()
{
	return m_txCustomPattern;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TX driver control

bool MultiLaneBERT::GetTxInvert(size_t i)
{
	return m_txInvert[i];
}

void MultiLaneBERT::SetTxInvert(size_t i, bool invert)
{
	if(invert)
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":INVERT 1");
	else
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":INVERT 0");

	m_txInvert[i] = invert;
}

vector<float> MultiLaneBERT::GetAvailableTxDriveStrengths(size_t /*i*/)
{
	vector<float> ret;
	ret.push_back(0.0);
	ret.push_back(0.1);
	ret.push_back(0.2);
	ret.push_back(0.3);
	ret.push_back(0.4);
	return ret;
}

float MultiLaneBERT::GetTxDriveStrength(size_t i)
{
	return m_txDrive[i];
}

void MultiLaneBERT::SetTxDriveStrength(size_t i, float drive)
{
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":SWING " + to_string((int)(drive*1000)));
	m_txDrive[i] = drive;
}

void MultiLaneBERT::SetTxEnable(size_t i, bool enable)
{
	if(enable)
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":ENABLE 1");
	else
		m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":ENABLE 0");

	m_txEnable[i] = enable;
}

bool MultiLaneBERT::GetTxEnable(size_t i)
{
	return m_txEnable[i];
}

float MultiLaneBERT::GetTxPreCursor(size_t i)
{
	return m_txPreCursor[i];
}

void MultiLaneBERT::SetTxPreCursor(size_t i, float precursor)
{
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PRECURSOR " + to_string((int)(precursor*100)));
	m_txPreCursor[i] = precursor;
}

float MultiLaneBERT::GetTxPostCursor(size_t i)
{
	return m_txPostCursor[i];
}

void MultiLaneBERT::SetTxPostCursor(size_t i, float postcursor)
{
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":POSTCURSOR " + to_string((int)(postcursor*100)));
	m_txPostCursor[i] = postcursor;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reference clock output

size_t MultiLaneBERT::GetRefclkOutMux()
{
	return m_refclkOutMux;
}

void MultiLaneBERT::SetRefclkOutMux(size_t i)
{
	//It seems that if you select a channel that's not currently locked, nothing changes
	switch(i)
	{
		case RX0_DIV8:
			m_transport->SendCommandQueued("CLKOUT RX0_DIV8");
			break;

		case RX0_DIV16:
			m_transport->SendCommandQueued("CLKOUT RX0_DIV16");
			break;

		case RX1_DIV8:
			m_transport->SendCommandQueued("CLKOUT RX1_DIV8");
			break;

		case RX1_DIV16:
			m_transport->SendCommandQueued("CLKOUT RX1_DIV16");
			break;

		case RX2_DIV8:
			m_transport->SendCommandQueued("CLKOUT RX2_DIV8");
			break;

		case RX2_DIV16:
			m_transport->SendCommandQueued("CLKOUT RX2_DIV16");
			break;

		case RX3_DIV8:
			m_transport->SendCommandQueued("CLKOUT RX3_DIV8");
			break;

		case RX3_DIV16:
			m_transport->SendCommandQueued("CLKOUT RX3_DIV16");
			break;

		case LO_DIV32_OR_80:
			m_transport->SendCommandQueued("CLKOUT LO_DIV32");	//or div80 in high speed mode
			break;

		case SERDES:
			m_transport->SendCommandQueued("CLKOUT SERDES");
			m_txCustomPattern = 0xaaaa;	//derpy
			break;

		default:
			break;
	}

	m_refclkOutMux = i;
}

vector<string> MultiLaneBERT::GetRefclkOutMuxNames()
{
	vector<string> ret;
	ret.push_back("RX0 CDR/8");
	ret.push_back("RX0 CDR/16");
	ret.push_back("RX1 CDR/8");
	ret.push_back("RX1 CDR/16");
	ret.push_back("RX2 CDR/8");
	ret.push_back("RX2 CDR/16");
	ret.push_back("RX3 CDR/8");
	ret.push_back("RX3 CDR/16");
	if(m_dataRate > 16000000000LL)
		ret.push_back("TX LO/80");
	else
	{
		ret.push_back("TX LO/32");

		//not available in high rate mode
		ret.push_back("SERDES");
	}
	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timebase

int64_t MultiLaneBERT::GetDataRate()
{
	return m_dataRate;
}

void MultiLaneBERT::SetDataRate(int64_t rate)
{
	m_transport->SendCommandQueued(string("RATE ") + to_string(rate));
	m_dataRate = rate;
}

vector<int64_t> MultiLaneBERT::GetAvailableDataRates()
{
	vector<int64_t> ret;

	ret.push_back( 8500000000LL);
	ret.push_back(10000000000LL);	//broken, gives garbage result
	ret.push_back(10312500000LL);
	ret.push_back(14025000000LL);
	ret.push_back(14062500000LL);
	ret.push_back(25000000000LL);
	ret.push_back(25781250000LL);
	ret.push_back(28050000000LL);
	//data file for 30 Gbps is present in the data directory
	//but doesn't seem to actually function

	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Data acquisition

bool MultiLaneBERT::GetRxCdrLockState(size_t i)
{
	return m_rxLock[i - m_rxChannelBase];
}

void MultiLaneBERT::MeasureHBathtub(size_t i)
{
	auto reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":HBATHTUB?");

	//Parse the reply
	auto data = explode(reply, ',');
	vector<float> values;
	float tmp;
	for(auto num : data)
	{
		sscanf(num.c_str(), "%f", &tmp);
		values.push_back(tmp);
	}

	if(values.size() < 256)
	{
		LogError("not enough data came back\n");
		return;
	}

	//Create the output waveform
	auto cap = dynamic_cast<SparseAnalogWaveform*>(GetChannel(i)->GetData(BERTInputChannel::STREAM_HBATHTUB));
	if(!cap)
	{
		cap = new SparseAnalogWaveform;
		GetChannel(i)->SetData(cap, BERTInputChannel::STREAM_HBATHTUB);
	}
	cap->PrepareForCpuAccess();
	cap->m_timescale = 1;
	cap->clear();

	/*
		Format of incoming data
			Timestamp (in ps relative to start of UI)
			BER (raw, not logarithmic)

		Up to 128 total pairs of points
			Points coming from left side of bathtub
			Dummy with timestamp of zero and BER of zero
			Points coming from right side of bathtub
			Zeroes as filler up to 128
	 */

	int state = 0;
	float last_time = 0;
	for(size_t j=0; j<128; j++)
	{
		float time = values[j*2];
		float ber = values[j*2 + 1];

		//If time goes backwards, we're dealing with a bug in mlBert_CalculateBathtubDualDirac
		//Discard this point
		if(time < last_time)
			continue;
		last_time = time;

		//If BER is invalid, we got hit by the bug also
		if(isinf(ber) || isnan(ber))
			continue;

		//Filler block? See if this is the end or what
		if(time < 0.001)
		{
			state ++;
			if(state == 2)
				break;
		}

		else
		{
			cap->m_offsets.push_back(round(time * 1000));	//convert ps to fs
			cap->m_samples.push_back(log10(ber));			//convert ber to logarithmic since display units are linear
		}
	}

	//Calculate durations
	for(size_t j=1; j<cap->m_offsets.size(); j++)
		cap->m_durations.push_back(cap->m_offsets[j] - cap->m_offsets[j-1]);
	cap->m_durations.push_back(1);

	//Time-shift entire waveform so zero is at the eye midpoint
	int64_t start = cap->m_offsets[0];
	int64_t end = cap->m_offsets[cap->m_offsets.size() - 1];
	cap->m_triggerPhase = -(start + end) / 2;
	for(size_t j=0; j<cap->m_offsets.size(); j++)
		cap->m_offsets[j] -= start;

	cap->MarkModifiedFromCpu();
}

void MultiLaneBERT::MeasureEye(size_t i)
{
	auto reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":EYE?");
	auto chan = dynamic_cast<BERTInputChannel*>(GetChannel(i));
	if(!chan)
		return;

	//Parse the reply
	auto data = explode(reply, ',');
	vector<float> values;
	float tmp;
	for(auto num : data)
	{
		sscanf(num.c_str(), "%f", &tmp);
		values.push_back(tmp);
	}

	if(values.size() < 32770)	//expect 32k plus x and y spacing
	{
		LogError("not enough data came back\n");
		return;
	}

	//Extract offsets
	int64_t dx_fs = round(values[0] * 1e3);
	float dy_v = values[1] * 1e-3;

	//Create the output waveform
	//Always 128 phases x 256 ADC codes and centered at 0V (since the input is AC coupled)
	//Make the texture 256 pixels wide due to normalization etc
	auto cap = new EyeWaveform(256, 256, 0.0, EyeWaveform::EYE_BER);
	cap->m_timescale = dx_fs;
	chan->SetData(cap, BERTInputChannel::STREAM_EYE);
	cap->PrepareForCpuAccess();

	//Set up metadata
	chan->SetVoltageRange(dy_v * 256, BERTInputChannel::STREAM_EYE);
	cap->m_uiWidth = dx_fs * 128;
	cap->m_saturationLevel = 3;

	//Copy the actual data
	auto accum = cap->GetAccumData();
	for(int y=0; y<256; y++)
	{
		for(int x=0; x<128; x++)
		{
			//Sample order coming off the BERT is right to left in X axis, then scanning bottom to top in Y
			double ber = values[y*128 + (127-x) + 2];

			//Rescale to generate fake hit count
			//Also need to rearrange so that we get the render-friendly eye pattern scopehal wants
			//(half a UI left and right of the center opening)
			if(x < 64)
				accum[y*256 + x + 192] = ber * 1e15;
			else
				accum[y*256 + x + 64] = ber * 1e15;
		}
	}
	cap->Normalize();
	cap->IntegrateUIs(1);	//have to put something here, but we don't have the true count value

	cap->MarkModifiedFromCpu();
}

bool MultiLaneBERT::AcquireData()
{
	//Poll CDR lock status
	for(int i=0; i<4; i++)
	{
		auto reply = m_transport->SendCommandQueuedWithReply(m_channels[i + m_rxChannelBase]->GetHwname() + ":LOCK?");
		m_rxLock[i] = (reply == "1");
	}

	return true;
}