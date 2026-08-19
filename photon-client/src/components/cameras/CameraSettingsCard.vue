<script setup lang="ts">
import PvSelect, { type SelectItem } from "@/components/common/pv-select.vue";
import PvDeleteModal from "@/components/common/pv-delete-modal.vue";
import PvNumberInput from "@/components/common/pv-number-input.vue";
import PvSwitch from "@/components/common/pv-switch.vue";
import { useCameraSettingsStore } from "@/stores/settings/CameraSettingsStore";
import { useStateStore } from "@/stores/StateStore";
import { computed, ref, watchEffect } from "vue";
import { type CameraSettingsChangeRequest, ValidQuirks } from "@/types/SettingTypes";
import { useTheme } from "vuetify";
import { axiosPost } from "@/lib/PhotonUtils";

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

const theme = useTheme();

const tempSettingsStruct = ref<CameraSettingsChangeRequest>({
  fov: useCameraSettingsStore().currentCameraSettings.fov.value,
  quirksToChange: Object.assign({}, useCameraSettingsStore().currentCameraSettings.cameraQuirks.quirks),
  robotToCamera: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
  includeInFusion: true
});
const focusMode = computed<boolean>({
  get: () => useCameraSettingsStore().isFocusMode,
  set: (v) =>
    useCameraSettingsStore().changeCurrentPipelineIndex(
      v ? -3 : useCameraSettingsStore().currentCameraSettings.lastPipelineIndex || 0,
      true
    )
});
const arducamSelectWrapper = computed<number>({
  get: () => {
    if (tempSettingsStruct.value.quirksToChange.ArduOV9281Controls) return 1;
    else if (tempSettingsStruct.value.quirksToChange.ArduOV2311Controls) return 2;
    else if (tempSettingsStruct.value.quirksToChange.ArduOV9782Controls) return 3;
    else return 0;
  },
  set: (v) => {
    switch (v) {
      case 1:
        tempSettingsStruct.value.quirksToChange.ArduOV9281Controls = true;
        tempSettingsStruct.value.quirksToChange.ArduOV2311Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV9782Controls = false;
        break;
      case 2:
        tempSettingsStruct.value.quirksToChange.ArduOV9281Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV2311Controls = true;
        tempSettingsStruct.value.quirksToChange.ArduOV9782Controls = false;
        break;
      case 3:
        tempSettingsStruct.value.quirksToChange.ArduOV9281Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV2311Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV9782Controls = true;
        break;
      default:
        tempSettingsStruct.value.quirksToChange.ArduOV9281Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV2311Controls = false;
        tempSettingsStruct.value.quirksToChange.ArduOV9782Controls = false;
        break;
    }
  }
});

const currentCameraIsArducam = computed<boolean>(
  () => useCameraSettingsStore().currentCameraSettings.cameraQuirks.quirks.ArduCamCamera
);

const settingsHaveChanged = (): boolean => {
  const a = tempSettingsStruct.value;
  const b = useCameraSettingsStore().currentCameraSettings;

  for (const q in ValidQuirks) {
    if (a.quirksToChange[q] != b.cameraQuirks.quirks[q]) return true;
  }

  return a.fov != b.fov.value || a.includeInFusion != b.includeInFusion || robotToCameraChanged();
};

const robotToCameraChanged = (): boolean => {
  const a = tempSettingsStruct.value.robotToCamera;
  const b = useCameraSettingsStore().currentCameraSettings.robotToCamera || {
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0
  };
  return (
    a.x !== b.x ||
    a.y !== b.y ||
    a.z !== b.z ||
    Math.abs(a.roll - b.roll * RAD2DEG) > 1e-6 ||
    Math.abs(a.pitch - b.pitch * RAD2DEG) > 1e-6 ||
    Math.abs(a.yaw - b.yaw * RAD2DEG) > 1e-6
  );
};

const resetTempSettingsStruct = () => {
  tempSettingsStruct.value.fov = useCameraSettingsStore().currentCameraSettings.fov.value;
  tempSettingsStruct.value.quirksToChange = Object.assign(
    {},
    useCameraSettingsStore().currentCameraSettings.cameraQuirks.quirks
  );
  const rtc = useCameraSettingsStore().currentCameraSettings.robotToCamera || {
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0
  };
  tempSettingsStruct.value.robotToCamera = {
    x: rtc.x,
    y: rtc.y,
    z: rtc.z,
    roll: rtc.roll * RAD2DEG,
    pitch: rtc.pitch * RAD2DEG,
    yaw: rtc.yaw * RAD2DEG
  };
  tempSettingsStruct.value.includeInFusion = useCameraSettingsStore().currentCameraSettings.includeInFusion !== false;
};

const saveCameraSettings = () => {
  const payload: CameraSettingsChangeRequest = {
    fov: tempSettingsStruct.value.fov,
    quirksToChange: tempSettingsStruct.value.quirksToChange,
    includeInFusion: tempSettingsStruct.value.includeInFusion,
    robotToCamera: {
      x: tempSettingsStruct.value.robotToCamera.x,
      y: tempSettingsStruct.value.robotToCamera.y,
      z: tempSettingsStruct.value.robotToCamera.z,
      roll: tempSettingsStruct.value.robotToCamera.roll * DEG2RAD,
      pitch: tempSettingsStruct.value.robotToCamera.pitch * DEG2RAD,
      yaw: tempSettingsStruct.value.robotToCamera.yaw * DEG2RAD
    }
  };
  useCameraSettingsStore()
    .updateCameraSettings(payload)
    .then((response) => {
      useStateStore().showSnackbarMessage({ color: "success", message: response.data.text || response.data });

      // Update the local settings cause the backend checked their validity. Assign is to deref value
      useCameraSettingsStore().currentCameraSettings.fov.value = tempSettingsStruct.value.fov;
      useCameraSettingsStore().currentCameraSettings.cameraQuirks.quirks = Object.assign(
        {},
        tempSettingsStruct.value.quirksToChange
      );
      useCameraSettingsStore().currentCameraSettings.robotToCamera = payload.robotToCamera;
      useCameraSettingsStore().currentCameraSettings.includeInFusion = payload.includeInFusion;
    })
    .catch((error) => {
      resetTempSettingsStruct();
      if (error.response) {
        useStateStore().showSnackbarMessage({
          color: "error",
          message: error.response.data.text || error.response.data
        });
      } else if (error.request) {
        useStateStore().showSnackbarMessage({
          color: "error",
          message: "Error while trying to process the request! The backend didn't respond."
        });
      } else {
        useStateStore().showSnackbarMessage({
          color: "error",
          message: "An error occurred while trying to process the request."
        });
      }
    });
};

watchEffect(() => {
  // Reset temp settings on remote camera settings change
  resetTempSettingsStruct();
});

const showDeleteCamera = ref(false);
const deleteThisCamera = () => {
  axiosPost("/utils/nukeOneCamera", "delete this camera", {
    cameraUniqueName: useStateStore().currentCameraUniqueName
  });
};
const wrappedCameras = computed<SelectItem[]>(() =>
  Object.keys(useCameraSettingsStore().cameras).map((cameraUniqueName) => ({
    name: useCameraSettingsStore().cameras[cameraUniqueName].nickname,
    value: cameraUniqueName
  }))
);
</script>

<template>
  <v-card class="mb-3 rounded-12" color="surface" dark>
    <v-card-title class="pb-0">Camera Settings</v-card-title>
    <v-card-text class="pt-3">
      <pv-select
        v-model="useStateStore().currentCameraUniqueName"
        label="Camera"
        :items="wrappedCameras"
        :select-cols="8"
      />
      <pv-number-input
        v-model="tempSettingsStruct.fov"
        :tooltip="
          !useCameraSettingsStore().currentCameraSettings.fov.managedByVendor
            ? 'Field of view (in degrees) of the camera measured across the diagonal of the frame, in a video mode which covers the whole sensor area.'
            : 'This setting is managed by a vendor'
        "
        label="Maximum Diagonal FOV"
        :disabled="useCameraSettingsStore().currentCameraSettings.fov.managedByVendor"
        :label-cols="4"
      />
      <pv-select
        v-show="currentCameraIsArducam"
        v-model="arducamSelectWrapper"
        label="Arducam Model"
        :items="[
          { name: 'None', value: 0, disabled: true },
          { name: 'OV9281', value: 1 },
          { name: 'OV2311', value: 2 },
          { name: 'OV9782', value: 3 }
        ]"
        :select-cols="8"
      />
      <pv-switch
        v-model="focusMode"
        tooltip="Enable Focus Mode to start focusing the lens on your camera"
        label="Focus Mode"
      ></pv-switch>
      <div class="text-subtitle-2 pt-4">Robot-to-camera (fusion)</div>
      <pv-number-input v-model="tempSettingsStruct.robotToCamera.x" label="X (m)" :step="0.001" :label-cols="4" />
      <pv-number-input v-model="tempSettingsStruct.robotToCamera.y" label="Y (m)" :step="0.001" :label-cols="4" />
      <pv-number-input v-model="tempSettingsStruct.robotToCamera.z" label="Z (m)" :step="0.001" :label-cols="4" />
      <pv-number-input v-model="tempSettingsStruct.robotToCamera.roll" label="Roll (deg)" :step="0.1" :label-cols="4" />
      <pv-number-input
        v-model="tempSettingsStruct.robotToCamera.pitch"
        label="Pitch (deg)"
        :step="0.1"
        :label-cols="4"
      />
      <pv-number-input v-model="tempSettingsStruct.robotToCamera.yaw" label="Yaw (deg)" :step="0.1" :label-cols="4" />
      <pv-switch
        v-model="tempSettingsStruct.includeInFusion"
        tooltip="Include this camera in multi-camera pose comparison streams"
        label="Include in pose fusion"
      ></pv-switch>
    </v-card-text>
    <v-card-text class="d-flex pt-0">
      <v-col cols="6" class="pa-0 pr-2">
        <v-btn
          block
          size="small"
          color="buttonActive"
          :disabled="!settingsHaveChanged()"
          :variant="theme.global.name.value === 'LightTheme' ? 'elevated' : 'outlined'"
          @click="saveCameraSettings"
        >
          <v-icon start size="large"> mdi-content-save </v-icon>
          Save Changes
        </v-btn>
      </v-col>
      <v-col cols="6" class="pa-0 pl-2">
        <v-btn
          block
          size="small"
          color="error"
          :variant="theme.global.name.value === 'LightTheme' ? 'elevated' : 'outlined'"
          @click="() => (showDeleteCamera = true)"
        >
          <v-icon start size="large"> mdi-trash-can-outline </v-icon>
          Delete Camera
        </v-btn>
      </v-col>
    </v-card-text>

    <pv-delete-modal
      v-model="showDeleteCamera"
      title="Delete Camera"
      :description="`Are you sure you want to delete the camera '${useCameraSettingsStore().currentCameraSettings.nickname}'? This action cannot be undone.`"
      :expected-confirmation-text="useCameraSettingsStore().currentCameraSettings.nickname"
      :on-confirm="deleteThisCamera"
    />
  </v-card>
</template>

<style scoped>
.v-divider {
  border-color: white !important;
}
</style>
