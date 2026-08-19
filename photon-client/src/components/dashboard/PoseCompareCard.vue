<script setup lang="ts">
import { computed } from "vue";
import { useStateStore } from "@/stores/StateStore";
import { useSettingsStore } from "@/stores/settings/GeneralSettingsStore";

const RAD2DEG = 180 / Math.PI;

const estimates = computed(() => useStateStore().poseCompareEstimates);
const enabled = computed(() => useSettingsStore().fusion.enabled);

const fmt = (v: number) => (Number.isFinite(v) ? v.toFixed(3) : "—");
const fmtDeg = (v: number) => (Number.isFinite(v) ? (v * RAD2DEG).toFixed(1) : "—");
</script>

<template>
  <v-card v-if="enabled" class="mb-3 rounded-12" color="surface">
    <v-card-title>Pose comparison</v-card-title>
    <v-card-text>
      <div v-if="estimates.length === 0" class="text-medium-emphasis">Waiting for 3D tag poses…</div>
      <v-table v-else density="compact">
        <thead>
          <tr>
            <th>Method</th>
            <th>Camera</th>
            <th>X (m)</th>
            <th>Y (m)</th>
            <th>Yaw (°)</th>
            <th>Tags</th>
            <th>ms</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="(est, i) in estimates" :key="i">
            <td>{{ est.method }}</td>
            <td>{{ est.camera || (est.camerasUsed || []).join(", ") }}</td>
            <td>{{ fmt(est.x) }}</td>
            <td>{{ fmt(est.y) }}</td>
            <td>{{ fmtDeg(est.yaw) }}</td>
            <td>{{ (est.fiducialIdsUsed || []).join(", ") }}</td>
            <td>{{ fmt(est.computeMs) }}</td>
          </tr>
        </tbody>
      </v-table>
    </v-card-text>
  </v-card>
</template>
