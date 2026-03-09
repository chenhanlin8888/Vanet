//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.subprojects.xindeluwangmoni;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple Applicationtestlayer2 extends DemoBaseApplLayer
{
    parameters:
        @class(xindeluwangmoni::Applicationtestlayer2);
        @display("i=block/app");
        double v2vRadius @unit(m) = default(150m);
        double baseErrorRate = default(0.01);
        double eavesdropRatio = default(0.0);
        double qberAlarm = default(0.11);
        double sampleFraction = default(0.25);
        double fEC = default(1.15);
        int Nraw = default(4096);

}