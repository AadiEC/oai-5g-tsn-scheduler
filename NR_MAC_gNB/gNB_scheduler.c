/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file gNB_scheduler.c
 * \brief gNB scheduler top level function operates on per subframe basis
 * \author  Navid Nikaein and Raymond Knopp, WEI-TAI CHEN
 * \date 2010 - 2014, 2018
 * \email: navid.nikaein@eurecom.fr, kroempa@gmail.com
 * \version 0.5
 * \company Eurecom, NTUST
 * @ingroup _mac

 */

#include "assertions.h"
#include <time.h>

#include "NR_MAC_COMMON/nr_mac_extern.h"
#include "NR_MAC_gNB/mac_proto.h"

#include "common/utils/nr/nr_common.h"

#include "common/utils/LOG/log.h"
#include "common/utils/nr/nr_common.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"

#include "openair2/X2AP/x2ap_eNB.h"

#include "nr_pdcp/nr_pdcp_oai_api.h"

#include "intertask_interface.h"

#include "executables/softmodem-common.h"
#include "nfapi/oai_integration/vendor_ext.h"
#include "executables/nr-softmodem.h"
FILE *cbs_tas_log_fp = NULL;

#include <errno.h>
#include <string.h>
void tas_cbs_init(gNB_MAC_INST *gNB); //Modify


const uint8_t nr_rv_round_map[4] = {0, 2, 3, 1};
//Modification
uint64_t get_current_time_us(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return(uint64_t)ts.tv_sec*1000000UL+ts.tv_nsec/1000UL;

}
// Corrected initialization function
void tas_cbs_init(gNB_MAC_INST *gNB) {
    int num_classes = 4;

    gNB->tas_cbs.num_traffic_classes = num_classes;

    // TAS Init with error checking
    gNB->tas_cbs.gcl_interval = 5000; // 5ms interval
    gNB->tas_cbs.current_gate_state = calloc(num_classes, sizeof(uint8_t));
    AssertFatal(gNB->tas_cbs.current_gate_state != NULL, "TAS gate state allocation failed");
    gNB->tas_cbs.gcl_length = 2;
    gNB->tas_cbs.gcl = malloc(gNB->tas_cbs.gcl_length * sizeof(NR_GCL_Entry_t));
    AssertFatal(gNB->tas_cbs.gcl != NULL, "GCL allocation failed");
    gNB->tas_cbs.gcl[0] = (NR_GCL_Entry_t){0, 2500, 0, 1}; // Open TC0 first 2.5ms
    gNB->tas_cbs.gcl[1] = (NR_GCL_Entry_t){2500, 5000, 0, 0}; // Close TC0 next 2.5ms

    // CBS Init with error checking
    gNB->tas_cbs.cbs_config = malloc(num_classes * sizeof(NR_CBS_Config_t));
    AssertFatal(gNB->tas_cbs.cbs_config != NULL, "CBS config allocation failed");
    for(int i=0; i<num_classes; i++) {
        gNB->tas_cbs.cbs_config[i].credit = 0;
        gNB->tas_cbs.cbs_config[i].idle_slope = 200 + i*50;
        gNB->tas_cbs.cbs_config[i].send_slope = -500 - i*100;
        gNB->tas_cbs.cbs_config[i].hi_limit = 10000 - i*2000;
        gNB->tas_cbs.cbs_config[i].lo_limit = -5000 + i*1000;
        gNB->tas_cbs.cbs_config[i].is_high_priority = (i == 0); // Mark TC0 as high-priority
        gNB->tas_cbs.cbs_config[i].burst_counter = 0;
    }

    gNB->tas_cbs.next_gcl_update = get_current_time_us() + gNB->tas_cbs.gcl_interval;
    cbs_tas_log_fp = fopen("cbs_tas_log.csv", "w");
if (cbs_tas_log_fp) {
  fprintf(cbs_tas_log_fp, "frame,slot,tc,credit,gate_state,burst_counter\n");
}

else {
        LOG_W(NR_MAC, "Warning: Couldn't open CBS+TAS log file!\n");
    }

}

// Helper to get scheduled bytes (if not present)

void clear_nr_nfapi_information(gNB_MAC_INST *gNB,
                                int CC_idP,
                                frame_t frameP,
                                sub_frame_t slotP,
                                nfapi_nr_dl_tti_request_t *DL_req,
                                nfapi_nr_tx_data_request_t *TX_req,
                                nfapi_nr_ul_dci_request_t *UL_dci_req)
{
  /* called below and in simulators, so we assume a lock but don't require it */

  NR_ServingCellConfigCommon_t *scc = gNB->common_channels->ServingCellConfigCommon;
  const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing];

  UL_tti_req_ahead_initialization(gNB, num_slots, CC_idP, frameP, slotP);

  nfapi_nr_dl_tti_pdcch_pdu_rel15_t **pdcch = (nfapi_nr_dl_tti_pdcch_pdu_rel15_t **)gNB->pdcch_pdu_idx[CC_idP];

  gNB->pdu_index[CC_idP] = 0;

  DL_req[CC_idP].SFN = frameP;
  DL_req[CC_idP].Slot = slotP;
  DL_req[CC_idP].dl_tti_request_body.nPDUs             = 0;
  DL_req[CC_idP].dl_tti_request_body.nGroup = 0;
  memset(pdcch, 0, sizeof(*pdcch) * MAX_NUM_CORESET);

  UL_dci_req[CC_idP].SFN = frameP;
  UL_dci_req[CC_idP].Slot = slotP;
  UL_dci_req[CC_idP].numPdus = 0;

  /* advance last round's future UL_tti_req to be ahead of current frame/slot */
  const int size = gNB->UL_tti_req_ahead_size;
  const int prev_slot = frameP * num_slots + slotP + size - 1;
  nfapi_nr_ul_tti_request_t *future_ul_tti_req = &gNB->UL_tti_req_ahead[CC_idP][prev_slot % size];
  future_ul_tti_req->SFN = (prev_slot / num_slots) % 1024;
  LOG_D(NR_MAC, "%d.%d UL_tti_req_ahead SFN.slot = %d.%d for index %d \n", frameP, slotP, future_ul_tti_req->SFN, future_ul_tti_req->Slot, prev_slot % size);
  /* future_ul_tti_req->Slot is fixed! */
  for (int i = 0; i < future_ul_tti_req->n_pdus; i++) {
    future_ul_tti_req->pdus_list[i].pdu_type = 0;
    future_ul_tti_req->pdus_list[i].pdu_size = 0;
  }
  future_ul_tti_req->n_pdus = 0;
  future_ul_tti_req->n_ulsch = 0;
  future_ul_tti_req->n_ulcch = 0;
  future_ul_tti_req->n_group = 0;

  TX_req[CC_idP].Number_of_PDUs = 0;
}

void clear_beam_information(NR_beam_info_t *beam_info, int frame, int slot, int mu)
{
  // for now we use the same logic of UL_tti_req_ahead
  // reset after 1 frame with the exception of 15kHz
  if(!beam_info->beam_allocation)
    return;
  // initialization done only once
  const int slots_per_frame = nr_slots_per_frame[mu];
  AssertFatal(beam_info->beam_allocation_size >= 0, "Beam information not initialized\n");
  int idx_to_clear = (frame * slots_per_frame + slot) / beam_info->beam_duration;
  idx_to_clear = (idx_to_clear + beam_info->beam_allocation_size - 1) % beam_info->beam_allocation_size;
  if (slot % beam_info->beam_duration == 0) {
    // resetting previous period allocation
    for (int i = 0; i < beam_info->beams_per_period; i++)
      beam_info->beam_allocation[i][idx_to_clear] = -1;
  }
}

bool is_xlsch_in_slot(uint64_t bitmap, sub_frame_t slot) {
  return (bitmap >> (slot % 64)) & 0x01;
}

/* the structure nfapi_nr_ul_tti_request_t is very big, let's copy only what is necessary */
static void copy_ul_tti_req(nfapi_nr_ul_tti_request_t *to, nfapi_nr_ul_tti_request_t *from)
{
  int i;

  to->header = from->header;
  to->SFN = from->SFN;
  to->Slot = from->Slot;
  to->n_pdus = from->n_pdus;
  to->rach_present = from->rach_present;
  to->n_ulsch = from->n_ulsch;
  to->n_ulcch = from->n_ulcch;
  to->n_group = from->n_group;

  for (i = 0; i < from->n_pdus; i++) {
    to->pdus_list[i].pdu_type = from->pdus_list[i].pdu_type;
    to->pdus_list[i].pdu_size = from->pdus_list[i].pdu_size;

    switch (from->pdus_list[i].pdu_type) {
      case NFAPI_NR_UL_CONFIG_PRACH_PDU_TYPE:
        to->pdus_list[i].prach_pdu = from->pdus_list[i].prach_pdu;
        break;
      case NFAPI_NR_UL_CONFIG_PUSCH_PDU_TYPE:
        to->pdus_list[i].pusch_pdu = from->pdus_list[i].pusch_pdu;
        break;
      case NFAPI_NR_UL_CONFIG_PUCCH_PDU_TYPE:
        to->pdus_list[i].pucch_pdu = from->pdus_list[i].pucch_pdu;
        break;
      case NFAPI_NR_UL_CONFIG_SRS_PDU_TYPE:
        to->pdus_list[i].srs_pdu = from->pdus_list[i].srs_pdu;
        break;
    }
  }

  for (i = 0; i < from->n_group; i++)
    to->groups_list[i] = from->groups_list[i];
}

void schedule_ues_for_class(int traffic_class, module_id_t module_id, frame_t frame, sub_frame_t slot, NR_Sched_Rsp_t *sched_info) {
    gNB_MAC_INST *gNB = RC.nrmac[module_id];
    if (!gNB) return;

    UE_iterator(gNB->UE_info.list, UE) {
        if (UE->traffic_class == traffic_class) {
            // Schedule this UE using the current context
            nr_schedule_ue_spec(module_id, frame, slot, &sched_info->DL_req, &sched_info->TX_req);
            
            // Debit CBS credit (only if not retransmission, as per your main scheduler logic)
            int32_t bytes_scheduled = UE->UE_sched_ctrl.sched_pdsch.tb_size;
            // Optional: Prevent over-debit if scheduled bytes > buffer status
            int32_t buf_bytes = UE->UE_sched_ctrl.rlc_status[0].bytes_in_buffer;
            if (bytes_scheduled > buf_bytes)
                bytes_scheduled = buf_bytes;
            gNB->tas_cbs.cbs_config[traffic_class].credit -= bytes_scheduled * 8; // bytes->bits
        }
    }
}


void gNB_dlsch_ulsch_scheduler(module_id_t module_idP, frame_t frame, sub_frame_t slot, NR_Sched_Rsp_t *sched_info)
{
  uint64_t now = get_current_time_us();
  protocol_ctxt_t ctxt = {0};
  PROTOCOL_CTXT_SET_BY_MODULE_ID(&ctxt, module_idP, ENB_FLAG_YES, NOT_A_RNTI, frame, slot,module_idP);

  gNB_MAC_INST *gNB = RC.nrmac[module_idP];
  NR_COMMON_channels_t *cc = gNB->common_channels;
  NR_ServingCellConfigCommon_t *scc = cc->ServingCellConfigCommon;

  NR_SCHED_LOCK(&gNB->sched_lock);

  clear_beam_information(&gNB->beam_info, frame, slot, *scc->ssbSubcarrierSpacing);
  
      // ==== High-Priority Preemption Pass ====
    for (int tc = 0; tc < gNB->tas_cbs.num_traffic_classes; tc++) {
        if (!gNB->tas_cbs.cbs_config[tc].is_high_priority) continue;
        if (gNB->tas_cbs.current_gate_state[tc] &&
            gNB->tas_cbs.cbs_config[tc].credit > 0) {
            // Schedule all UEs in this high-priority class
            schedule_ues_for_class(tc, module_idP, frame, slot, sched_info);

            LOG_I(NR_MAC, "Preempt: Scheduled high-priority TC%d at %lums", tc, now);
            // Preempt lower classes by returning immediately
            NR_SCHED_UNLOCK(&gNB->sched_lock);
            return;
        }
    }
    // ==== End Preemption Pass ====

if ((frame % 100 == 0) && (slot == 0)) { // Every 100ms
  for (int tc = 0; tc < gNB->tas_cbs.num_traffic_classes; tc++) {
    LOG_I(NR_MAC,
      "[CBS+TAS] Frame %d Slot %d TC%d | Credit: %d | Gate: %d | BurstCounter: %u\n",
      frame, slot, tc,
      gNB->tas_cbs.cbs_config[tc].credit,
      gNB->tas_cbs.current_gate_state[tc],
      gNB->tas_cbs.cbs_config[tc].burst_counter);
  }
}



  ////  CBS + TAS LOGIC Placement /////////////////////

if(now >= gNB->tas_cbs.next_gcl_update) {
    // Reset burst counters at start of every GCL interval
    for (int tc = 0; tc < gNB->tas_cbs.num_traffic_classes; tc++) {
        gNB->tas_cbs.cbs_config[tc].burst_counter = 0;
    }

    uint64_t current_interval = (now - gNB->tas_cbs.next_gcl_update + gNB->tas_cbs.gcl_interval) % gNB->tas_cbs.gcl_interval;
    memset(gNB->tas_cbs.current_gate_state, 0, gNB->tas_cbs.num_traffic_classes);
    for(int i=0; i<gNB->tas_cbs.gcl_length; i++) {
        if(current_interval >= gNB->tas_cbs.gcl[i].start_time &&
           current_interval < gNB->tas_cbs.gcl[i].end_time) {
            gNB->tas_cbs.current_gate_state[gNB->tas_cbs.gcl[i].traffic_class] = gNB->tas_cbs.gcl[i].gate_state;
        }
    }
    gNB->tas_cbs.next_gcl_update += gNB->tas_cbs.gcl_interval;
}

const uint64_t elapsed_us = NR_SLOT_DURATION_US(*scc->ssbSubcarrierSpacing);
for(int tc=0; tc<gNB->tas_cbs.num_traffic_classes; tc++) {
    if(gNB->tas_cbs.current_gate_state[tc]) {
        gNB->tas_cbs.cbs_config[tc].credit += gNB->tas_cbs.cbs_config[tc].send_slope * elapsed_us;
    } else {
        gNB->tas_cbs.cbs_config[tc].credit += gNB->tas_cbs.cbs_config[tc].idle_slope * elapsed_us;
    }
    //New:
    if (gNB->tas_cbs.cbs_config[tc].credit < 
    gNB->tas_cbs.cbs_config[tc].lo_limit) {
  gNB->tas_cbs.cbs_config[tc].credit =
      gNB->tas_cbs.cbs_config[tc].lo_limit;
  }
  if (gNB->tas_cbs.cbs_config[tc].credit >
      gNB->tas_cbs.cbs_config[tc].hi_limit) {
    gNB->tas_cbs.cbs_config[tc].credit =
        gNB->tas_cbs.cbs_config[tc].hi_limit;
  }

    //gNB->tas_cbs.cbs_config[tc].credit = MAX(gNB->tas_cbs.cbs_config[tc].credit, gNB->tas_cbs.cbs_config[tc].lo_limit);
    //gNB->tas_cbs.cbs_config[tc].credit = MIN(gNB->tas_cbs.cbs_config[tc].credit, gNB->tas_cbs.cbs_config[tc].hi_limit);
}

// === CBS Bounded Delay Enforcement ===
int R = 1000000000; // Transmitter rate in bits/sec (adjust to match your gNB capacity)
for (int tc = 0; tc < gNB->tas_cbs.num_traffic_classes; tc++) {
    if (gNB->tas_cbs.cbs_config[tc].is_high_priority)
        continue; // no burst limit for HP traffic

    int S = gNB->tas_cbs.cbs_config[tc].idle_slope;
    int L = 1500 * 8; // Max shaper packet size in bits
    int F = 1500 * 8; // Max interfering packet size in bits

    uint32_t max_burst = (L * R + S * F) / (R - S);
    if (gNB->tas_cbs.cbs_config[tc].burst_counter >= max_burst) {
        LOG_D(NR_MAC, "TC%d: Burst limit hit (%u >= %u), delaying shaped traffic\n",
              tc, gNB->tas_cbs.cbs_config[tc].burst_counter, max_burst);
        gNB->tas_cbs.current_gate_state[tc] = 0; // Force gate closed for this cycle
    }
}

///// CBS + TAS LOGIC END /////////////////////////

  gNB->frame = frame;
  start_meas(&gNB->eNB_scheduler);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_DLSCH_ULSCH_SCHEDULER,VCD_FUNCTION_IN);

  /* send tick to RLC, PDCP, and X2AP every ms */
  if ((slot & ((1 << *scc->ssbSubcarrierSpacing) - 1)) == 0) {
    void nr_rlc_tick(int frame, int subframe);
    void nr_pdcp_tick(int frame, int subframe);
    nr_rlc_tick(frame, slot >> *scc->ssbSubcarrierSpacing);
    nr_pdcp_tick(frame, slot >> *scc->ssbSubcarrierSpacing);
    if (is_x2ap_enabled())
      x2ap_trigger();
  }

  for (int CC_id = 0; CC_id < MAX_NUM_CCs; CC_id++) {
    int num_beams = 1;
    if(gNB->beam_info.beam_allocation)
      num_beams = gNB->beam_info.beams_per_period;
    // clear vrb_maps
    for (int i = 0; i < num_beams; i++)
      memset(cc[CC_id].vrb_map[i], 0, sizeof(uint16_t) * MAX_BWP_SIZE);
    // clear last scheduled slot's content (only)!
    const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing];
    const int size = gNB->vrb_map_UL_size;
    const int prev_slot = frame * num_slots + slot + size - 1;
    for (int i = 0; i < num_beams; i++) {
      uint16_t *vrb_map_UL = cc[CC_id].vrb_map_UL[i];
      memcpy(&vrb_map_UL[prev_slot % size * MAX_BWP_SIZE], &gNB->ulprbbl, sizeof(uint16_t) * MAX_BWP_SIZE);
    }
    clear_nr_nfapi_information(gNB, CC_id, frame, slot, &sched_info->DL_req, &sched_info->TX_req, &sched_info->UL_dci_req);
  }

  if ((slot == 0) && (frame & 127) == 0) {
    char stats_output[32656] = {0};
    dump_mac_stats(gNB, stats_output, sizeof(stats_output), true);
    LOG_I(NR_MAC, "Frame.Slot %d.%d\n%s\n", frame, slot, stats_output);
  }

  nr_mac_update_timers(module_idP, frame, slot);

  // ====== AADI UE Scheduling Loop
  UE_iterator(gNB->UE_info.list, UE){
    int tc = UE->traffic_class;
    //HARQ Process
    // Add this block at the start of each UE's scheduling loop
	bool has_harq_retx = false;
	for (int h = 0; h < NR_MAX_HARQ_PROCESSES; h++) {
    	if (UE->UE_sched_ctrl.harq_processes[h].is_waiting) {
        	has_harq_retx = true;
        break;
    	}
	}
     
    // HARQ END
    
    // 1. TAS Gate Check: Skip if gate is closed
if (!gNB->tas_cbs.current_gate_state[tc] && !has_harq_retx) {
    LOG_D(NR_MAC, "UE %d: TAS gate closed for TC%d\n", UE->rnti, tc);
    continue;
}
if (gNB->tas_cbs.cbs_config[tc].credit < 0 && !has_harq_retx) {
    LOG_D(NR_MAC, "UE %d: Insufficient credit (%d) for TC%d\n",
          UE->rnti, gNB->tas_cbs.cbs_config[tc].credit, tc);
    continue;
}

     //if (nr_schedule_ue_spec(module_idP, frame, slot, &sched_info->DL_req, &sched_info->TX_req)) {
      nr_schedule_ue_spec(module_idP, frame, slot,
                    &sched_info->DL_req,
                    &sched_info->TX_req);   
     // 4. Debit credit based on actual scheduled bytes (not full TBS)
        int32_t bytes_scheduled = UE->UE_sched_ctrl.sched_pdsch.tb_size;
        // Optional: Prevent over-debit if scheduled bytes > buffer status
        //if (bytes_scheduled > UE->dl_buffer_status)
            //bytes_scheduled = UE->dl_buffer_status;
	int32_t buf_bytes = UE->UE_sched_ctrl.rlc_status[0].bytes_in_buffer;
	if (bytes_scheduled > buf_bytes)
	    bytes_scheduled = buf_bytes;
	if (!has_harq_retx)
	    gNB->tas_cbs.cbs_config[tc].credit -= bytes_scheduled * 8; // bytes->bits
	 // bytes->bits
	 if (!has_harq_retx && !gNB->tas_cbs.cbs_config[tc].is_high_priority)
    gNB->tas_cbs.cbs_config[tc].burst_counter += bytes_scheduled * 8;
        LOG_D(NR_MAC, "Scheduled UE %d (TC%d): Debited %d bits, New credit=%d\n",
              UE->rnti, tc, bytes_scheduled*8, gNB->tas_cbs.cbs_config[tc].credit);
    //}
}
  /// AADI UE Scheduling ENDS


  // This schedules MIB
  schedule_nr_mib(module_idP, frame, slot, &sched_info->DL_req);

  // This schedules SIB1
  // SIB19 will be scheduled if ntn_Config_r17 is initialized
  if (IS_SA_MODE(get_softmodem_params())) {
    schedule_nr_sib1(module_idP, frame, slot, &sched_info->DL_req, &sched_info->TX_req);
    if (cc->sib19)
      schedule_nr_sib19(module_idP, frame, slot, &sched_info->DL_req, &sched_info->TX_req, cc->sib19_bcch_length, cc->sib19_bcch_pdu);
  }

  // This schedule PRACH if we are not in phy_test mode
  if (get_softmodem_params()->phy_test == 0) {
    /* we need to make sure that resources for PRACH are free. To avoid that
       e.g. PUSCH has already been scheduled, make sure we schedule before
       anything else: below, we simply assume an advance one frame (minus one
       slot, because otherwise we would allocate the current slot in
       UL_tti_req_ahead), but be aware that, e.g., K2 is allowed to be larger
       (schedule_nr_prach will assert if resources are not free). */
    const sub_frame_t n_slots_ahead = nr_slots_per_frame[*scc->ssbSubcarrierSpacing] - 1 + get_NTN_Koffset(scc);
    const frame_t f = (frame + (slot + n_slots_ahead) / nr_slots_per_frame[*scc->ssbSubcarrierSpacing]) % 1024;
    const sub_frame_t s = (slot + n_slots_ahead) % nr_slots_per_frame[*scc->ssbSubcarrierSpacing];
    schedule_nr_prach(module_idP, f, s);
  }

  // Schedule CSI-RS transmission
  nr_csirs_scheduling(module_idP, frame, slot, nr_slots_per_frame[*scc->ssbSubcarrierSpacing], &sched_info->DL_req);

  // Schedule CSI measurement reporting
  nr_csi_meas_reporting(module_idP, frame, slot);

  nr_schedule_srs(module_idP, frame, slot);

  // This schedule RA procedure if not in phy_test mode
  // Otherwise consider 5G already connected
  if (get_softmodem_params()->phy_test == 0) {
    nr_schedule_RA(module_idP, frame, slot, &sched_info->UL_dci_req, &sched_info->DL_req, &sched_info->TX_req);
  }

  // This schedules the DCI for Uplink and subsequently PUSCH
  nr_schedule_ulsch(module_idP, frame, slot, &sched_info->UL_dci_req);

  // This schedules the DCI for Downlink and PDSCH
  start_meas(&gNB->schedule_dlsch);
  nr_schedule_ue_spec(module_idP, frame, slot, &sched_info->DL_req, &sched_info->TX_req);
  stop_meas(&gNB->schedule_dlsch);

  nr_sr_reporting(gNB, frame, slot);

  nr_schedule_pucch(gNB, frame, slot);

  /* TODO: we copy from gNB->UL_tti_req_ahead[0][current_index], ie. CC_id == 0,
   * is more than 1 CC supported?
   */
  AssertFatal(MAX_NUM_CCs == 1, "only 1 CC supported\n");
  const int current_index = ul_buffer_index(frame, slot, *scc->ssbSubcarrierSpacing, gNB->UL_tti_req_ahead_size);
  copy_ul_tti_req(&sched_info->UL_tti_req, &gNB->UL_tti_req_ahead[0][current_index]);

  stop_meas(&gNB->eNB_scheduler);
  NR_SCHED_UNLOCK(&gNB->sched_lock);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_DLSCH_ULSCH_SCHEDULER,VCD_FUNCTION_OUT);
}
