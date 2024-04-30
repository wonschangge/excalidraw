import React, { useEffect, useRef } from "react";
import {
  getDropdownMenuItemClassName,
  useHandleDropdownMenuItemClick,
} from "./common";
import MenuItemContent from "./DropdownMenuItemContent";
import { useExcalidrawAppState } from "../App";
import { THEME } from "../../constants";
import type { ValueOf } from "../../utility-types";

const DropdownMenuItem = ({
  icon,
  value,
  children,
  shortcut,
  className,
  hovered,
  selected,
  textStyle,
  onSelect,
  onClick,
  ...rest
}: {
  icon?: JSX.Element;
  value?: string | number | undefined;
  onSelect?: (event: Event) => void;
  children: React.ReactNode;
  shortcut?: string;
  hovered?: boolean;
  selected?: boolean;
  textStyle?: React.CSSProperties,
  className?: string;
} & Omit<React.ButtonHTMLAttributes<HTMLButtonElement>, "onSelect">) => {
  const handleClick = useHandleDropdownMenuItemClick(onClick, onSelect);
  const ref = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    if (hovered) {
      ref.current?.scrollIntoView({ block: "nearest" });
    }
  }, [hovered]);

  return (
    <button
      {...rest}
      ref={ref}
      value={value}
      onClick={handleClick}
      className={getDropdownMenuItemClassName(className, selected, hovered)}
      title={rest.title ?? rest["aria-label"]}
    >
      <MenuItemContent textStyle={textStyle} icon={icon} shortcut={shortcut}>
        {children}
      </MenuItemContent>
    </button>
  );
};
DropdownMenuItem.displayName = "DropdownMenuItem";

export const DropDownMenuItemBadgeType = {
  GREEN: "green",
  BLUE: "blue",
} as const;

export const DropDownMenuItemBadge = ({
  type = DropDownMenuItemBadgeType.BLUE,
  children,
}: {
  type?: ValueOf<typeof DropDownMenuItemBadgeType>;
  children: React.ReactNode;
}) => {
  const { theme } = useExcalidrawAppState();
  const style = {
    display: "inline-flex",
    marginLeft: "auto",
    padding: "2px 4px",
    borderRadius: 6,
    fontSize: 9,
    fontFamily: "Cascadia, monospace",
    border: theme === THEME.LIGHT ? "1.5px solid white" : "none",
  };

  switch (type) {
    case DropDownMenuItemBadgeType.GREEN:
      Object.assign(style, {
        backgroundColor: "var(--background-color-badge)",
        color: "var(--color-badge)",
      });
      break;
    case DropDownMenuItemBadgeType.BLUE:
    default:
      Object.assign(style, {
        background: "var(--color-promo)",
        color: "var(--color-surface-lowest)",
      });
  }

  return (
    <div className="DropDownMenuItemBadge" style={style}>
      {children}
    </div>
  );
};
DropDownMenuItemBadge.displayName = "DropdownMenuItemBadge";

DropdownMenuItem.Badge = DropDownMenuItemBadge;

export default DropdownMenuItem;
