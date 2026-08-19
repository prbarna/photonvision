<script setup lang="ts">
import { ref, watchEffect } from "vue";
import PvSwitch from "@/components/common/pv-switch.vue";
import PvNumberInput from "@/components/common/pv-number-input.vue";
import { useSettingsStore } from "@/stores/settings/GeneralSettingsStore";
import { useStateStore } from "@/stores/StateStore";
import { useTheme } from "vuetify";
import type { MultiCameraFusionConfig } from "@/types/SettingTypes";

const theme = useTheme();
const temp = ref<MultiCameraFusionConfig>({ ...useSettingsStore().fusion });

const resetTemp = () => {
  temp.value = { ...useSettingsStore().fusion };
};

watchEffect(() => {
  resetTemp();
});

const settingsHaveChanged = (): boolean => {
  const a = useSettingsStore().fusion;
  const b = temp.value;
  return (
    a.enabled !== b.enabled ||
    a.maxDtMs !== b.maxDtMs ||
    a.minCameras !== b.minCameras ||
    a.baseline !== b.baseline ||
    a.weightedAverage !== b.weightedAverage ||
    a.jointPnp !== b.jointPnp ||
    a.writeCsv !== b.writeCsv
  );
};

const save = async () => {
  try {
    const response = await useSettingsStore().updateFusionSettings(temp.value);
    useStateStore().showSnackbarMessage({
      message: response.data.text || response.data,
      color: "success"
    });
    useSettingsStore().fusion = { ...temp.value };
  } catch (error: any) {
    resetTemp();
    useStateStore().showSnackbarMessage({
      color: "error",
      message: error.response?.data?.text || error.response?.data || "Failed to save fusion settings"
    });
  }
};
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface">
    <v-card-title>Multi-camera pose comparison</v-card-title>
    <v-card-text>
      <pv-switch
        v-model="temp.enabled"
        label="Enable pose comparison"
        tooltip="Run baseline, weighted-average, and joint PnP on the coprocessor and log all poses"
      />
      <pv-number-input v-model="temp.maxDtMs" label="Max Δt (ms)" :step="1" :label-cols="4" />
      <pv-number-input v-model="temp.minCameras" label="Min cameras" :step="1" :label-cols="4" />
      <pv-switch v-model="temp.baseline" label="Baseline (per-camera MultiTag)" />
      <pv-switch v-model="temp.weightedAverage" label="Weighted pose average" />
      <pv-switch v-model="temp.jointPnp" label="Joint constrained PnP" />
      <pv-switch
        v-model="temp.writeCsv"
        label="Write CSV log"
        tooltip="Append poses to photonvision_config/logs/pose-compare-*.csv"
      />
    </v-card-text>
    <v-card-text class="d-flex pt-0">
      <v-btn
        block
        size="small"
        color="buttonActive"
        :disabled="!settingsHaveChanged()"
        :variant="theme.global.name.value === 'LightTheme' ? 'elevated' : 'outlined'"
        @click="save"
      >
        <v-icon start size="large">mdi-content-save</v-icon>
        Save Fusion Settings
      </v-btn>
    </v-card-text>
  </v-card>
</template>
