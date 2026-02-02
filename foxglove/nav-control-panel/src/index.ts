import { ExtensionContext } from "@foxglove/extension";

import { initNavControlPanel } from "./NavControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Navigation Control",
    initPanel: initNavControlPanel,
  });
}
