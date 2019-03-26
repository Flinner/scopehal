/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief Implementation of OscilloscopeChannel
 */

#include "scopehal.h"
#include "OscilloscopeChannel.h"
#include "ChannelRenderer.h"
#include "AnalogRenderer.h"
#include "DigitalRenderer.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

OscilloscopeChannel::OscilloscopeChannel(
	Oscilloscope* scope,
	string hwname,
	OscilloscopeChannel::ChannelType type,
	string color,
	int width,
	size_t index,
	bool physical)
	: m_displaycolor(color)
	, m_displayname(hwname)
	, m_timescale(1e-2)
	, m_scope(scope)
	, m_data(NULL)
	, m_type(type)
	, m_hwname(hwname)
	, m_width(width)
	, m_procedural(false)
	, m_index(index)
	, m_physical(physical)
{
}

OscilloscopeChannel::~OscilloscopeChannel()
{
	delete m_data;
	m_data = NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers for calling scope functions

bool OscilloscopeChannel::IsEnabled()
{
	return m_scope->IsChannelEnabled(m_index);
}

void OscilloscopeChannel::Enable()
{
	m_scope->EnableChannel(m_index);
}

void OscilloscopeChannel::Disable()
{
	m_scope->DisableChannel(m_index);
}

OscilloscopeChannel::CouplingType OscilloscopeChannel::GetCoupling()
{
	return m_scope->GetChannelCoupling(m_index);
}

void OscilloscopeChannel::SetCoupling(CouplingType type)
{
	m_scope->SetChannelCoupling(m_index, type);
}

double OscilloscopeChannel::GetAttenuation()
{
	return m_scope->GetChannelAttenuation(m_index);
}

void OscilloscopeChannel::SetAttenuation(double atten)
{
	m_scope->SetChannelAttenuation(m_index, atten);
}

int OscilloscopeChannel::GetBandwidthLimit()
{
	return m_scope->GetChannelBandwidthLimit(m_index);
}

void OscilloscopeChannel::SetBandwidthLimit(int mhz)
{
	m_scope->SetChannelBandwidthLimit(m_index, mhz);
}

double OscilloscopeChannel::GetVoltageRange()
{
	return m_scope->GetChannelVoltageRange(m_index);
}

void OscilloscopeChannel::SetVoltageRange(double range)
{
	return m_scope->SetChannelVoltageRange(m_index, range);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

OscilloscopeChannel::ChannelType OscilloscopeChannel::GetType()
{
	return m_type;
}

string OscilloscopeChannel::GetHwname()
{
	return m_hwname;
}

CaptureChannelBase* OscilloscopeChannel::GetData()
{
	return m_data;
}

void OscilloscopeChannel::SetData(CaptureChannelBase* pNew)
{
	delete m_data;
	m_data = pNew;
}

int OscilloscopeChannel::GetWidth()
{
	return m_width;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Factory methods

ChannelRenderer* OscilloscopeChannel::CreateRenderer()
{
	switch(m_type)
	{
	case CHANNEL_TYPE_DIGITAL:
		return new DigitalRenderer(this);

	case CHANNEL_TYPE_ANALOG:
		return new AnalogRenderer(this);

	//complex channels must be procedural (and override this)
	default:
	case CHANNEL_TYPE_COMPLEX:
		return NULL;
	}
}
