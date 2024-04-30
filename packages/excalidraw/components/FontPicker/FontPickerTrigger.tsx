import * as Popover from "@radix-ui/react-popover";
import { useMemo } from "react";
import { ButtonIcon } from "../ButtonIcon";
import { FontFamilyNormalIcon } from "../icons";
import type { FontFamilyValues } from "../../element/types";
import { t } from "../../i18n";
import { isCustomFont } from "./FontPicker";

interface FontPickerTriggerProps {
  selectedFontFamily: FontFamilyValues | null;
}

export const FontPickerTrigger = ({
  selectedFontFamily,
}: FontPickerTriggerProps) => {
  const isTriggerActive = useMemo(
    () => Boolean(selectedFontFamily && isCustomFont(selectedFontFamily)),
    [selectedFontFamily],
  );

  return (
    <Popover.Trigger asChild>
      {/* Empty div as trigger so it's stretched 100% due to different button sizes */}
      <div>
        <ButtonIcon
          standalone
          icon={FontFamilyNormalIcon}
          title={t("labels.showFonts")}
          className="properties-trigger"
          testId={"font-family-show-fonts"}
          active={isTriggerActive}
          // no-op
          onClick={() => {}}
        />
      </div>
    </Popover.Trigger>
  );
};
