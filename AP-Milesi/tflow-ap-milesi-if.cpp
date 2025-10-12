#include "../tflow-build-cfg.hpp"

#include "../tflow-ap.hpp"
#include "../tflow-ctrl.hpp"
#include "tflow-ap-milesi-cfg.hpp"

#include "tflow-ap-milesi.hpp"

/*
 * Interface file to link user's specific algorithm with TFlowProcess
 * User have to define the createAlgoInstance() function and Algorithm's 
 * configuration.
 */

TFlowAP* TFlowAP::createAPInstance(MainContextPtr context)
{
    return (TFlowAP*)(new TFlowMilesi(context, &tflow_milesi_cfg.cmd_flds_cfg_milesi));
}

static struct TFlowCtrlUI::uictrl ui_group_ap_def = {
    .type = TFlowCtrlUI::UICTRL_TYPE::GROUP,
};

TFlowAP::tflow_cfg_ap cmd_flds_cfg_ap  = {
    TFLOW_CMD_HEAD("ap_head"),
    .tflow_ap = {"Milesi", TFlowCtrl::CFT_REF, 0, {.ref = &tflow_milesi_cfg.cmd_flds_cfg_milesi.head}, &ui_group_ap_def},
    TFLOW_CMD_EOMSG
};
