import { ExtensionContext } from "@foxglove/extension";

import { initGamepadRelayPanel } from "./GamepadRelayPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Gamepad Relay",
    initPanel: initGamepadRelayPanel,
  });
}
