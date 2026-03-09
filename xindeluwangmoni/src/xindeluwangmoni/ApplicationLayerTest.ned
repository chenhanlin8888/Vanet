package org.car2x.veins.subprojects.xindeluwangmoni;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple ApplicationLayerTest extends DemoBaseApplLayer
{
    parameters:
        @class(xindeluwangmoni::ApplicationLayerTest);  // 关键：绑定到你的C++类

        // V2V 仅用到的参数
        double v2vRadius @unit(m) = default(150m);
        double baseErrorRate = default(0.01);
        double eavesdropRatio = default(0.0);
        double qberAlarm = default(0.11);
        double sampleFraction = default(0.125);
        double fEC = default(1.15);
        int    Nraw = default(4096);
        bool useSARG04 = default(false);
}
