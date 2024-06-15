/* eslint-disable prettier/prettier */
import React from "react";
import { AppProps, AppState } from "../types";
import { getDefaultAppState } from "../appState";
import { STORAGE_KEYS } from "../../excalidraw-app/app_constants";
import { getDateTime } from "../utils";
import { t } from "../i18n";

class TitleInput extends React.Component<AppProps, AppState> {
  static drawName: string;

  constructor(props: AppProps) {
    super(props);
    const defaultAppState = getDefaultAppState();
    const {
      viewModeEnabled = false,
      zenModeEnabled = false,
      gridModeEnabled = false,
      objectsSnapModeEnabled = false,
      theme = defaultAppState.theme,
      name = defaultAppState.name,
    } = props;
    this.state = {
      ...defaultAppState,
      theme,
      isLoading: true,
      offsetLeft: 0,
      offsetTop: 0,
      viewModeEnabled,
      zenModeEnabled,
      objectsSnapModeEnabled,
      gridSize: gridModeEnabled ? 20 : null,
      name: name.replace(/-.*/, ""),
      width: window.innerWidth,
      height: window.innerHeight,
    };

    TitleInput.drawName = name;
  }

  changeName = (name: string) => {
    const localName = this.state.name;
    const createTime = localName.match(/(\d+-?)*$/g)?.[0] || getDateTime();
    const localState = JSON.parse(
      localStorage.getItem(STORAGE_KEYS.LOCAL_STORAGE_APP_STATE)!,
    );
    localState.name = `${name}-${createTime}`;
    localStorage.setItem(
      STORAGE_KEYS.LOCAL_STORAGE_APP_STATE,
      JSON.stringify(localState),
    );

    TitleInput.drawName = localState.name;
  };

  public render(): React.ReactNode {
    const isDarkTheme = this.state.theme === "dark";

    return (
      <div
        style={{
          zIndex: 2,
          fontSize: "14px",
          color: isDarkTheme ? "var(--color-gray-60)" : "var(--color-gray-50)",
          width: "100%",
          whiteSpace: "nowrap",
          textOverflow: "ellipsis",
        }}
      >
        <input
          id="title-input"
          autoFocus
          placeholder="标题"
          defaultValue={this.state.name}
          onBlur={(e) => {
            let value = e.target.value.trim();
            if (!value.trim()) {
              value = t("labels.untitled");
              e.target.value = value;
            }
            this.changeName(value);
          }}
          style={{
            background: "rgba(0, 0, 0, 0)",
            zIndex: 2,
            border: "0",
            display: "block",
            fontFamily: "Assistant",
            fontSize: "14px",
            color: "var(--color-gray-80)",
            overflow: "hidden",
            outline: "none",
            textAlign: "center",
            borderRadius: "4px",
            width: "100%",
          }}
        />

      </div>
    );
  }
}

export default TitleInput;
