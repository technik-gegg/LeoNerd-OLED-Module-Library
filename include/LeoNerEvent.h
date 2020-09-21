/**
 * Library for LeoNerd's OLED Module
 * 
 * Copyright (C) 2020 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#ifndef _LEONERD_EVENT_H
#define _LEONERD_EVENT_H 1

#include "ButtonState.h"

class LeoNerdEvent {
public:
    LeoNerdEvent() { EvtType = None; State = Open; Value = 0;}
    LeoNerdEvent(EventType event, ButtonState state, uint8_t value = 0) { EvtType = event; State = state; Value = value; }
    volatile EventType      EvtType;
    volatile ButtonState    State;
    volatile uint8_t        Value;
};
#endif