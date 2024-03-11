/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libcola - A library providing force-directed network layout using the
 *           stress-majorization method subject to separation constraints.
 *
 * Copyright (C) 2015  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Tim Dwyer
 */

#include "libcola/PseudoRandom.hxx"

namespace cola {

PseudoRandom::PseudoRandom(double s)
    : a(21'4013)
    , c(253'1011)
    , m(21'4748'3648)
    , range(3'2767)
    , seed(s)
{
}

double PseudoRandom::getNext(void)
{
    seed = (seed * a + c) % m;
    return (seed >> 16) / range;
}

double PseudoRandom::getNextBetween(double min, double max)
{
    return min + getNext() * (max - min);
}

}  // namespace cola
