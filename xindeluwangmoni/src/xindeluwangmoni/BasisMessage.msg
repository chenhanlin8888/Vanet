import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;
import veins.modules.application.traci.TraCIDemo11pMessage;

namespace xindeluwangmoni;

packet BasisMessage extends veins::BaseFrame1609_4 {
    int qubitId;    // 与 QubitMessage 一一对应
    int basis;      // Bob 测量时选择的基：0=Z，1=X
}