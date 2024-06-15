import { CapacitorConfig } from "@capacitor/cli";

const config: CapacitorConfig = {
  appId: "com.H5.excalidraw",
  appName: "excalidraw",
  webDir: "excalidraw-app/build",
  server: {
    androidScheme: "https",
  },
};

export default config;
