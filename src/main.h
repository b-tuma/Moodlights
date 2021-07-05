// ##### BEGIN GPL LICENSE BLOCK #####
//
//  This program is free software: you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation, either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// ##### END GPL LICENSE BLOCK #####
//
// Copyright (C) 2021  Bruno Tuma <bruno.tuma@outlook.com>

#include <Arduino.h>

struct HsvColor
{
    HsvColor(uint8_t h, uint8_t s, uint8_t v, uint8_t w) :
        H(h), S(s), V(v), W(w)
    {
    };

    HsvColor()
    {
    };

    uint8_t H;
    uint8_t S;
    uint8_t V;
    uint8_t W;
};

