import { ExtensionContext } from "@foxglove/extension";

import { initHardwareSafetyPanel } from "./HardwareSafetyPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Hardware Safety",
    initPanel: initHardwareSafetyPanel,
  });
}
