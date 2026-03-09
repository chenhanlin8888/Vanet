import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;
import veins.modules.application.traci.TraCIDemo11pMessage;

namespace xindeluwangmoni;

packet QubitMessage extends veins::BaseFrame1609_4 {
    int qubitId;    // 用来标识第几个比特
    int bit;        // Alice 准备的 0/1 值
    int basis;      // Alice 准备的基：0=Z，1=X
}