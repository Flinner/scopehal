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
#include "LycheeOscilloscope.h"
#include "OscilloscopeChannel.h"
#include "TestWaveformSource.h"
#include "EdgeTrigger.h"

#include <cinttypes>
#include <cstdio>
#include <iostream>

#ifdef _WIN32
#include <chrono>
#include <thread>
#endif

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Construction / destruction

LycheeOscilloscope::LycheeOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_triggerArmed(false)
	, m_triggerWasLive(false)
	, m_triggerOneShot(false)
{
	transport->EnableRateLimiting(std::chrono::milliseconds(1));
	transport->FlushRXBuffer();
	const int nchans = 2;
	for(int i = 0; i < nchans; i++)
	{
		//Hardware name of the channel
		string chname = string("ch") + to_string(i + 1);

		//Color the channels based on Rigol's standard color sequence (yellow-cyan-red-blue)
		string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#00ffff";
				break;

			case 2:
				color = "#ff00ff";
				break;

			case 3:
				color = "#336699";
				break;
		}

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this, chname, color, Unit(Unit::UNIT_FS), Unit(Unit::UNIT_VOLTS), Stream::STREAM_TYPE_ANALOG, i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();
	}
	m_analogChannelCount = nchans;

	//Add the external trigger input
	// m_extTrigChannel = new OscilloscopeChannel(
	// 	this, "EX", "", Unit(Unit::UNIT_FS), Unit(Unit::UNIT_VOLTS), Stream::STREAM_TYPE_TRIGGER, m_channels.size());
	// m_channels.push_back(m_extTrigChannel);
	// m_extTrigChannel->SetDefaultDisplayName();

	//make sure all setup commands finish before we proceed
	m_transport->FlushCommandQueue();
}

LycheeOscilloscope::~LycheeOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Accessors

unsigned int LycheeOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t LycheeOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Device interface functions

string LycheeOscilloscope::GetDriverNameInternal()
{
	return "sad";
}

void LycheeOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelAttenuations.clear();
	m_channelCouplings.clear();
	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelsEnabled.clear();
	m_channelBandwidthLimits.clear();

	m_srateValid = false;
	m_mdepthValid = false;
	m_triggerOffsetValid = false;

	delete m_trigger;
	m_trigger = NULL;
}

bool LycheeOscilloscope::IsChannelEnabled(size_t i)
{
	//ext trigger should never be displayed
	// if(i == m_extTrigChannel->GetIndex())
	// 	return false;

	//TODO: handle digital channels, for now just claim they're off
	if(i >= m_analogChannelCount)
		return false;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":" + m_channels[i]->GetHwname() + ":DISPQ"));
	std::cout << reply << std::endl;
	if(reply == "0")
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelsEnabled[i] = false;
		return false;
	}
	else
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelsEnabled[i] = true;
		return true;
	}
}

void LycheeOscilloscope::EnableChannel(size_t i)
{
	m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":DISP ON");
	// invalidate channel enable cache until confirmed on next IsChannelEnabled
	m_channelsEnabled.erase(i);
}

void LycheeOscilloscope::DisableChannel(size_t i)
{
	m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":DISP OFF");
	// invalidate channel enable cache until confirmed on next IsChannelEnabled
	m_channelsEnabled.erase(i);
}

vector<OscilloscopeChannel::CouplingType> LycheeOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	//TODO: some higher end models do have 50 ohm inputs... which ones?
	//ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	ret.push_back(OscilloscopeChannel::COUPLE_GND);
	return ret;
}

OscilloscopeChannel::CouplingType LycheeOscilloscope::GetChannelCoupling(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":" + m_channels[i]->GetHwname() + ":COUPQ"));

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	if(reply == "AC")
		m_channelCouplings[i] = OscilloscopeChannel::COUPLE_AC_1M;
	else if(reply == "DC")
		m_channelCouplings[i] = OscilloscopeChannel::COUPLE_DC_1M;
	else /* if(reply == "GND") */
		m_channelCouplings[i] = OscilloscopeChannel::COUPLE_GND;
	return m_channelCouplings[i];
}

void LycheeOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	bool valid = true;
	switch(type)
	{
		case OscilloscopeChannel::COUPLE_AC_1M:
			m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":COUP AC");
			break;

		case OscilloscopeChannel::COUPLE_DC_1M:
			m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":COUP DC");
			break;

		case OscilloscopeChannel::COUPLE_GND:
			m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":COUP GND");
			break;

		default:
			LogError("Invalid coupling for channel\n");
			valid = false;
	}

	if(valid)
	{
		lock_guard<recursive_mutex> lock2(m_cacheMutex);
		m_channelCouplings[i] = type;
	}
}

double LycheeOscilloscope::GetChannelAttenuation(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}

	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":" + m_channels[i]->GetHwname() + ":PROBQ"));

	double atten;
	sscanf(reply.c_str(), "%lf", &atten);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = atten;
	return atten;
}

void LycheeOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	bool valid = true;
	switch((
		int)(atten * 10000 +
			 0.1))	  //+ 0.1 in case atten is for example 0.049999 or so, to round it to 0.05 which turns to an int of 500
	{
		case 1:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.0001");
			break;
		case 2:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.0002");
			break;
		case 5:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.0005");
			break;
		case 10:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.001");
			break;
		case 20:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.002");
			break;
		case 50:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.005");
			break;
		case 100:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.01");
			break;
		case 200:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.02");
			break;
		case 500:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.05");
			break;
		case 1000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.1");
			break;
		case 2000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.2");
			break;
		case 5000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 0.5");
			break;
		case 10000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 1");
			break;
		case 20000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 2");
			break;
		case 50000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 5");
			break;
		case 100000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 10");
			break;
		case 200000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 20");
			break;
		case 500000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 50");
			break;
		case 1000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 100");
			break;
		case 2000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 200");
			break;
		case 5000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 500");
			break;
		case 10000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 1000");
			break;
		case 20000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 2000");
			break;
		case 50000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 5000");
			break;
		case 100000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 10000");
			break;
		case 200000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 20000");
			break;
		case 500000000:
			m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":PROB 50000");
			break;
		default:
			LogError("Invalid attenuation for channel\n");
			valid = false;
	}

	if(valid)
	{
		lock_guard<recursive_mutex> lock2(m_cacheMutex);
		m_channelAttenuations[i] = (int)(atten * 10000 + 0.1) * 0.0001;
	}
}

vector<unsigned int> LycheeOscilloscope::GetChannelBandwidthLimiters(size_t /*i*/)
{
	vector<unsigned int> ret;
	ret = {20, 0};
	return ret;
}

unsigned int LycheeOscilloscope::GetChannelBandwidthLimit(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelBandwidthLimits.find(i) != m_channelBandwidthLimits.end())
			return m_channelBandwidthLimits[i];
	}

	auto reply = Trim(m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":BWLQ"));

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	if(reply == "20M")
		m_channelBandwidthLimits[i] = 20;
	if(reply == "100M")
		m_channelBandwidthLimits[i] = 100;
	if(reply == "200M")
		m_channelBandwidthLimits[i] = 200;
	else
		m_channelBandwidthLimits[i] = m_bandwidth;
	return m_channelBandwidthLimits[i];
}

void LycheeOscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	bool valid = true;

	switch(m_bandwidth)
	{
		case 70:
		case 100:
			if((limit_mhz <= 20) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 20M");
			else
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL OFF");
			break;
		case 200:
			if((limit_mhz <= 20) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 20M");
			else if((limit_mhz <= 100) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 100M");
			else
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL OFF");
			break;
		case 350:
			if((limit_mhz <= 20) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 20M");
			else if((limit_mhz <= 100) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 100M");
			else if((limit_mhz <= 200) & (limit_mhz != 0))
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL 200M");
			else
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BWL OFF");
			break;
		default:
			LogError("Invalid model number\n");
			valid = false;
	}

	if(valid)
	{
		lock_guard<recursive_mutex> lock2(m_cacheMutex);
		if(limit_mhz == 0)
			m_channelBandwidthLimits[i] = m_bandwidth;	  // max
		else if(limit_mhz <= 20)
			m_channelBandwidthLimits[i] = 20;
		else if(m_bandwidth == 70)
			m_channelBandwidthLimits[i] = 70;
		else if((limit_mhz <= 100) | (m_bandwidth == 100))
			m_channelBandwidthLimits[i] = 100;
		else if((limit_mhz <= 200) | (m_bandwidth == 200))
			m_channelBandwidthLimits[i] = 200;
		else
			m_channelBandwidthLimits[i] = m_bandwidth;	  // 350 MHz
	}
}

float LycheeOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	string reply;
	// reply = Trim(m_transport->SendCommandQueuedWithReply(":" + m_channels[i]->GetHwname() + ":RANGEQ"));

	float range = 5.0;
	sscanf(reply.c_str(), "%f", &range);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	range = 8 * range;
	m_channelVoltageRanges[i] = range;

	return range;
}

void LycheeOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	{
		lock_guard<recursive_mutex> lock2(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":RANGE " + to_string(range));
}

OscilloscopeChannel* LycheeOscilloscope::GetExternalTrigger()
{
	//FIXME
	return nullptr;
}

float LycheeOscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":" + m_channels[i]->GetHwname() + ":OFFSQ"));

	float offset;
	sscanf(reply.c_str(), "%f", &offset);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void LycheeOscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":OFFS " + to_string(offset));

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
}

Oscilloscope::TriggerMode LycheeOscilloscope::PollTrigger()
{
	// auto stat = Trim(m_transport->SendCommandQueuedWithReply(":TRIG:STATQ"));

	// if(stat != "STOP")
	// 	m_triggerWasLive = true;

	// if(stat == "TD")
	// 	return TRIGGER_MODE_TRIGGERED;
	// else if(stat == "RUN")
	// 	return TRIGGER_MODE_RUN;
	// else if(stat == "WAIT")
	// 	return TRIGGER_MODE_WAIT;
	// else if(stat == "AUTO")
	// 	return TRIGGER_MODE_AUTO;
	// else
	// {
	// 	//The "TD" state is not sticky on Sad scopes, unlike the equivalent LeCroy status register bit.
	// 	//The scope will go from "run" to "stop" state on trigger with only a momentary pass through "TD".
	// 	//If we armed the trigger recently and we're now stopped, this means we must have triggered.
	// 	if(m_triggerArmed && m_triggerWasLive)
	// 	{
	// 		m_triggerArmed = false;
	// 		m_triggerWasLive = false;
	// 		return TRIGGER_MODE_TRIGGERED;
	// 	}

	// 	//Nope, we're actually stopped
	// 	return TRIGGER_MODE_STOP;
	// }

	if(m_triggerArmed)
		return TRIGGER_MODE_TRIGGERED;
	else
		return TRIGGER_MODE_STOP;
}

// bool LycheeOscilloscope::AcquireData_OLD()
// {
// LogDebug("Acquiring data\n");

// lock_guard<recursive_mutex> lock(m_transport->GetMutex());
// LogIndenter li;

// //Grab the analog waveform data
// int unused1 = 0;
// int unused2 = 0;
// size_t npoints = 1000;
// int unused3 = 0;
// double sec_per_sample = 1e-6;
// double xorigin = 0;
// double xreference = 1.0;
// double yincrement = 1.0;
// double yorigin = 0;
// double yreference = 0;
// size_t maxpoints = 250 * 1000;
// maxpoints = GetSampleDepth();	 //You can use 250E6 points too, but it is very slow
// unsigned char* temp_buf = new unsigned char[maxpoints + 1];
// map<int, vector<UniformAnalogWaveform*>> pending_waveforms;
// for(size_t i = 0; i < 1; i++)
// {
// 	if(!IsChannelEnabled(i))
// 		continue;

// 	LogDebug("Channel %zu\n", i);

// 	int64_t fs_per_sample = 1;

// 	m_transport->SendCommandQueued(string("WAV:SOUR ") + m_channels[i]->GetHwname());
// 	std::cout << "HORRA 1" << std::endl;

// 	//This is basically the same function as a LeCroy WAVEDESC, but much less detailed
// 	auto reply = Trim(m_transport->SendCommandQueuedWithReply("WAV:PREQ"));
// 	LogDebug("Preamble = %s\n", reply.c_str());	   // TODO: comment
// 	// sscanf(reply.c_str(),
// 	// 	"%d,%d,%zu,%d,%lf,%lf,%lf,%lf,%lf,%lf",
// 	// 	&unused1,
// 	// 	&unused2,
// 	// 	&npoints,
// 	// 	&unused3,
// 	// 	&sec_per_sample,
// 	// 	&xorigin,
// 	// 	&xreference,
// 	// 	&yincrement,
// 	// 	&yorigin,
// 	// 	&yreference);
// 	fs_per_sample = round(sec_per_sample * FS_PER_SECOND);
// 	LogDebug("X: %zu points, %f origin, ref %f fs/sample %ld\n", npoints, xorigin, xreference, fs_per_sample);
// 	LogDebug("Y: %f inc, %f origin, %f ref\n", yincrement, yorigin, yreference);

// 	//Set up the capture we're going to store our data into
// 	auto cap = AllocateAnalogWaveform(m_nickname + "." + GetChannel(i)->GetHwname());
// 	cap->Resize(0);
// 	cap->m_timescale = fs_per_sample;
// 	cap->m_triggerPhase = 0;
// 	cap->m_startTimestamp = time(NULL);
// 	double t = GetTime();
// 	cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

// 	//Downloading the waveform is a pain in the butt, because we can only pull 250K points at a time! (Unless you have a MSO5)
// 	// for(size_t npoint = 0; npoint < npoints;)
// 	// {
// 	//Ask for the data block
// 	m_transport->SendCommandQueued("*WAI");
// 	m_transport->SendCommandQueued("WAV:DATAQ");
// 	m_transport->FlushCommandQueue();
// 	m_transport->ReadRawData(1000, temp_buf);	 //trailing newline after data block

// 	double ydelta = yorigin + yreference;
// 	cap->Resize(2000);
// 	cap->PrepareForCpuAccess();
// 	// for(size_t j = 0; j < header_blocksize; j++)
// 	for(size_t j = 0; j < 1000; j++)
// 	{
// 		float v = (static_cast<float>(temp_buf[j]) - ydelta) * yincrement;
// 		// LogDebug("V = %.3f, temp=%d, delta=%f, inc=%f\n", v, temp_buf[j], ydelta, yincrement);
// 		cap->m_samples[j] = i & 0xF;
// 	}
// 	cap->MarkSamplesModifiedFromCpu();

// 	//Done, update the data
// 	if(cap)
// 		pending_waveforms[i].push_back(cap);
// 	std::cout << "HORRA" << std::endl;

// 	m_pendingWaveformsMutex.lock();
// 	size_t num_pending = 1;	   //TODO: segmented capture support
// 	for(size_t k = 0; k < num_pending; k++)
// 	{
// 		SequenceSet s;
// 		for(size_t j = 0; j < m_analogChannelCount; j++)
// 		{
// 			if(pending_waveforms.count(j) > 0)
// 				s[GetOscilloscopeChannel(j)] = pending_waveforms[j][k];
// 		}
// 		m_pendingWaveforms.push_back(s);
// 	}
// 	m_pendingWaveformsMutex.unlock();

// 	//If this was a one-shot trigger we're no longer armed
// 	if(m_triggerOneShot)
// 		m_triggerArmed = false;

// 	//Re-arm the trigger if not in one-shot mode
// 	if(!m_triggerOneShot)
// 	{
// 		m_transport->SendCommandQueued(":SING");
// 		m_transport->SendCommandQueued("*WAI");
// 		m_triggerArmed = true;
// 	}

// 	LogDebug("Acquisition done\n");
// }
// return true;
// }

bool LycheeOscilloscope::AcquireData()
{
	//cap waveform rate at 50 wfm/s to avoid saturating cpu
	std::this_thread::sleep_for(std::chrono::microseconds(20 * 1000));

	//Sweeping frequency
	//Signal degradations
	//Generate waveforms

	auto depth = GetSampleDepth();
	int64_t sampleperiod = FS_PER_SECOND / m_srate;
	WaveformBase* waveforms[2] = {NULL};
	map<int, vector<UniformAnalogWaveform*>> pending_waveforms;

#pragma omp parallel for
	for(int i = 0; i < 2; i++)
	{
		if(!m_channelsEnabled[i])
			continue;

		auto ret = new UniformAnalogWaveform("NoisySine");
		float samples_per_cycle = 1e6 * 1.0 / sampleperiod;
		float radians_per_sample = 2 * M_PI / samples_per_cycle;
		//sin is +/- 1, so need to divide amplitude by 2 to get scaling factor
		float scale = 1.0 / 2;

		auto ret_2 = new UniformAnalogWaveform("NoisySine");
		float samples_per_cycle_2 = 1e6 * 1.0 / sampleperiod;
		float radians_per_sample_2 = 2 * M_PI / samples_per_cycle_2;
		//sin is +/- 1, so need to divide amplitude by 2 to get scaling factor
		float scale_2 = 1.0 / 2;
		switch(i)
		{
			case 0:
				ret->m_timescale = sampleperiod;
				ret->Resize(depth);

				for(size_t j = 0; j < depth; j++)
					ret->m_samples[j] = scale * sinf(j * radians_per_sample + 0.0);

				waveforms[i] = ret;
				break;

			case 1:
				ret_2->m_timescale = sampleperiod;
				ret_2->Resize(depth);

				for(size_t j = 0; j < depth; j++)
					ret_2->m_samples[j] = scale_2 * sinf(j * radians_per_sample_2 + 0.0);

				waveforms[i] = ret_2;
				break;
			default:
				break;
		}

		waveforms[i]->MarkModifiedFromCpu();
	}

	SequenceSet s;
	for(int i = 0; i < 2; i++)
		s[GetOscilloscopeChannel(i)] = waveforms[i];

	//Timestamp the waveform(s)
	double now = GetTime();
	time_t start = now;
	double tfrac = now - start;
	int64_t fs = tfrac * FS_PER_SECOND;
	for(auto it : s)
	{
		auto wfm = it.second;
		if(!wfm)
			continue;

		wfm->m_startTimestamp = start;
		wfm->m_startFemtoseconds = fs;
		wfm->m_triggerPhase = 0;
	}

	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);
	m_pendingWaveformsMutex.unlock();

	if(m_triggerOneShot)
		m_triggerArmed = false;

	return true;
}

// bool LycheeOscilloscope::AcquireData()
// {
// 	const int depth_max = 1000000;	  // FIXME: Allow configuration.
// 	static uint8_t waveform[depth_max];

// 	lock_guard<recursive_mutex> lock2(m_mutex);

// 	string reply;

// 	/* Get Waveform Data. */
// 	int depth;
// 	// reply = Trim(m_transport->SendCommandQueuedWithReply("SAMP:DEPTHQ"));
// 	// sscanf(reply.c_str(), "%d", &depth);
// 	depth = 1000;
// 	// m_transport->ReadRawData(depth, waveform);
// 	for(int i = 0; i < depth; i++)
// 		waveform[i] = i & 0xF;

// 	/* Get Waveform Timescale. */
// 	int timescale;
// 	// reply = Trim(m_transport->SendCommandQueuedWithReply("SAMP:TIMQ"));
// 	sscanf(reply.c_str(), "%d", &timescale);
// 	timescale = 1;

// 	/* Prepare Waveform. */
// 	map<int, vector<UniformAnalogWaveform*>> pending_waveforms;
// 	for(size_t j = 0; j < m_analogChannelCount; j++)
// 	{
// 		/* Set Waveform Timescale/Timestamp. */
// 		auto cap = AllocateAnalogWaveform(m_nickname + "." + GetChannel(j)->GetHwname());
// 		double t = GetTime();
// 		cap->m_timescale = FS_PER_SECOND / m_srate;
// 		cap->m_triggerPhase = 0;
// 		cap->m_startTimestamp = floor(t);
// 		cap->m_startFemtoseconds = (t - cap->m_startTimestamp) * FS_PER_SECOND;

// 		/* Re-Scale Samples and add them to the Waveform */
// 		float fullscale = GetChannelVoltageRange(0, 0);
// 		float scale = fullscale / 256.0f;
// 		float offset = GetChannelOffset(0, 0);
// 		cap->Resize(depth / m_analogChannelCount);
// 		cap->PrepareForCpuAccess();
// 		for(size_t i = 0; i < depth / m_analogChannelCount; i++)
// 		{
// 			// cap->m_offsets[i] = i;
// 			// cap->m_durations[i] = 1;
// 			cap->m_samples[i] = ((waveform[m_analogChannelCount * j + i] - 128.0f) * scale) + offset;
// 		}
// 		cap->MarkSamplesModifiedFromCpu();

// 		/* Done, Push Waveform */
// 		lock_guard<recursive_mutex> lock(m_mutex);
// 		pending_waveforms[j].push_back(cap);
// 	}

// 	/* Now that we have all of the pending waveforms, save them in sets across all channels. */
// 	SequenceSet s;
// 	for(size_t j = 0; j < m_analogChannelCount; j++)
// 	{
// 		if(IsChannelEnabled(j))
// 			s[m_channels[j]] = pending_waveforms[j][0];
// 	}
// 	m_pendingWaveforms.push_back(s);

// 	//If this was a one-shot trigger we're no longer armed
// 	if(m_triggerOneShot)
// 		m_triggerArmed = false;

// 	//Re-arm the trigger if not in one-shot mode
// 	if(!m_triggerOneShot)
// 	{
// 		m_transport->SendCommandQueued(":SING");
// 		m_transport->SendCommandQueued("*WAI");
// 		m_triggerArmed = true;
// 	}

// 	LogDebug("Acquisition done\n");

// 	return true;
// }

void LycheeOscilloscope::Start()
{
	//LogDebug("Start single trigger\n");
	m_transport->SendCommandQueued(":SING");
	m_transport->SendCommandQueued("*WAI");
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void LycheeOscilloscope::StartSingleTrigger()
{
	m_transport->SendCommandQueued(":TRIG:EDGE:SWE SING");
	m_transport->SendCommandQueued(":RUN");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void LycheeOscilloscope::Stop()
{
	m_transport->SendCommandQueued(":STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void LycheeOscilloscope::ForceTrigger()
{
	// m_transport->SendCommandQueued(":TFOR");
	StartSingleTrigger();
}

bool LycheeOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<uint64_t> LycheeOscilloscope::GetSampleRatesNonInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	ret = {
		100,
		200,
		500,
		1000,
		2000,
		5000,
		10 * 1000,
		20 * 1000,
		50 * 1000,
		100 * 1000,
		200 * 1000,
		500 * 1000,
		1 * 1000 * 1000,
		2 * 1000 * 1000,
		5 * 1000 * 1000,
		10 * 1000 * 1000,
		20 * 1000 * 1000,
		50 * 1000 * 1000,
		100 * 1000 * 1000,
		200 * 1000 * 1000,
		500 * 1000 * 1000,
		1 * 1000 * 1000 * 1000,
		2 * 1000 * 1000 * 1000,
	};
	return ret;
}

vector<uint64_t> LycheeOscilloscope::GetSampleRatesInterleaved()
{
	//FIXME
	vector<uint64_t> ret = {};
	LogError("LycheeOscilloscope::GetSampleRatesInterleaved not implemented for this model\n");
	return ret;
}

set<Oscilloscope::InterleaveConflict> LycheeOscilloscope::GetInterleaveConflicts()
{
	//FIXME
	set<Oscilloscope::InterleaveConflict> ret;
	LogError("LycheeOscilloscope::GetInterleaveConflicts not implemented for this model\n");
	return ret;
}

vector<uint64_t> LycheeOscilloscope::GetSampleDepthsNonInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	ret = {
		1000,
		10 * 1000,
		100 * 1000,
		1000 * 1000,
		10 * 1000 * 1000,
		25 * 1000 * 1000,
	};
	return ret;
}

vector<uint64_t> LycheeOscilloscope::GetSampleDepthsInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	LogError("LycheeOscilloscope::GetSampleDepthsInterleaved not implemented for this model\n");
	return ret;
}

uint64_t LycheeOscilloscope::GetSampleRate()
{
	if(m_srateValid)
		return m_srate;

	// FIXME
	// auto ret = Trim(m_transport->SendCommandQueuedWithReply(":ACQ:SRATQ"));

	uint64_t rate = 50e9;
	// sscanf(ret.c_str(), "%" PRIu64, &rate);
	m_srate = rate;
	m_srateValid = true;
	return rate;
}

uint64_t LycheeOscilloscope::GetSampleDepth()
{
	// if(m_mdepthValid)
	// 	return m_mdepth;

	// auto ret = Trim(m_transport->SendCommandQueuedWithReply(":ACQ:MDEP?"));

	// double depth;
	// sscanf(ret.c_str(), "%lf", &depth);
	// m_mdepth = (uint64_t)depth;
	// m_mdepthValid = true;
	// return m_mdepth;
	return 100e3;
}

void LycheeOscilloscope::SetSampleDepth(uint64_t depth)
{
	// // The MSO5 series will only process a sample depth setting if the oscilloscope is in auto or normal mode.
	// // It's frustrating, but to accommodate, we'll grab the current mode and status for restoration later, then stick the
	// // scope into auto mode
	// string trigger_sweep_mode = m_transport->SendCommandQueuedWithReply(":TRIG:SWE?");
	// string trigger_status = m_transport->SendCommandQueuedWithReply(":TRIG:STAT?");
	// m_transport->SendCommandQueued(":TRIG:SWE AUTO");
	// m_transport->SendCommandQueued(":RUN");
	// switch(depth)
	// {
	// 	case 1000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 1k");
	// 		break;
	// 	case 10000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 10k");
	// 		break;
	// 	case 100000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 100k");
	// 		break;
	// 	case 1000000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 1M");
	// 		break;
	// 	case 10000000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 10M");
	// 		break;
	// 	case 25000000:
	// 		m_transport->SendCommandQueued("ACQ:MDEP 25M");
	// 		break;
	// 	case 50000000:
	// 		if(m_opt200M)
	// 			m_transport->SendCommandQueued("ACQ:MDEP 50M");
	// 		else
	// 			LogError("Invalid memory depth for channel: %" PRIu64 "\n", depth);
	// 		break;
	// 	case 100000000:
	// 		//m_transport->SendCommandQueued("ACQ:MDEP 100M");
	// 		LogError("Invalid memory depth for channel: %" PRIu64 "\n", depth);
	// 		break;
	// 	case 200000000:
	// 		//m_transport->SendCommandQueued("ACQ:MDEP 200M");
	// 		LogError("Invalid memory depth for channel: %" PRIu64 "\n", depth);
	// 		break;
	// 	default:
	// 		LogError("Invalid memory depth for channel: %" PRIu64 "\n", depth);
	// }
	// m_transport->SendCommandQueued(":TRIG:SWE " + trigger_sweep_mode);
	// // This is a little hairy - do we want to stop the instrument again if it was stopped previously? Probably?
	// if(trigger_status == "STOP")
	// 	m_transport->SendCommandQueued(":STOP");
}

void LycheeOscilloscope::SetSampleRate(uint64_t rate)
{
	// //FIXME, you can set :TIMebase:SCALe
	// m_mdepthValid = false;
	// double sampletime = GetSampleDepth() / (double)rate;

	// m_transport->SendCommandQueued(string(":TIM:SCAL ") + to_string(sampletime / 10));

	// m_srateValid = false;
	// m_mdepthValid = false;
}

void LycheeOscilloscope::SetTriggerOffset(int64_t offset)
{
	// double offsetval = (double)offset / FS_PER_SECOND;

	// m_transport->SendCommandQueued(string(":TIM:MAIN:OFFS ") + to_string(offsetval));
}

int64_t LycheeOscilloscope::GetTriggerOffset()
{
	if(m_triggerOffsetValid)
		return m_triggerOffset;

	auto ret = Trim(m_transport->SendCommandQueuedWithReply(":TIM:MAIN:OFFSQ"));

	double offsetval;
	sscanf(ret.c_str(), "%lf", &offsetval);
	m_triggerOffset = (uint64_t)(offsetval * FS_PER_SECOND);
	m_triggerOffsetValid = true;
	return m_triggerOffset;
}

bool LycheeOscilloscope::IsInterleaving()
{
	return false;
}

bool LycheeOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void LycheeOscilloscope::PullTrigger()
{
	//Figure out what kind of trigger is active.
	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":TRIG:MODEQ"));
	// std::cout << "REPLAY: " << reply << std::endl;
	if(reply == "EDGE")
		PullEdgeTrigger();

	//Unrecognized trigger type
	else
	{
		LogWarning("Unknown trigger type \"%s\"\n", reply.c_str());
		m_trigger = NULL;
		return;
	}
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void LycheeOscilloscope::PullEdgeTrigger()
{
	//Clear out any triggers of the wrong type
	if((m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	//Source
	auto reply = Trim(m_transport->SendCommandQueuedWithReply(":TRIG:EDGE:SOURQ"));
	auto chan = GetOscilloscopeChannelByHwName(reply);
	et->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	reply = Trim(m_transport->SendCommandQueuedWithReply(":TRIG:EDGE:LEVQ"));
	et->SetLevel(stof(reply));

	//Edge slope
	reply = Trim(m_transport->SendCommandQueuedWithReply(":TRIG:EDGE:SLOPEQ"));
	if(reply == "POS")
		et->SetType(EdgeTrigger::EDGE_RISING);
	else if(reply == "NEG")
		et->SetType(EdgeTrigger::EDGE_FALLING);
	else if(reply == "RFAL")
		et->SetType(EdgeTrigger::EDGE_ANY);
}

void LycheeOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void LycheeOscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	//Type
	m_transport->SendCommandQueued(":TRIG:MODE EDGE");

	//Source
	m_transport->SendCommandQueued(":TRIG:EDGE:SOUR " + trig->GetInput(0).m_channel->GetHwname());

	//Level
	m_transport->SendCommandQueued(string("TRIG:EDGE:LEV") + to_string(trig->GetLevel()));

	//Slope
	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			m_transport->SendCommandQueued(":TRIG:EDGE:SLOPE POS");
			break;
		case EdgeTrigger::EDGE_FALLING:
			m_transport->SendCommandQueued(":TRIG:EDGE:SLOPE NEG");
			break;
		case EdgeTrigger::EDGE_ANY:
			m_transport->SendCommandQueued(":TRIG:EDGE:SLOPE RFAL");
			break;
		default:
			LogWarning("Unknown edge type\n");
			return;
	}
}
