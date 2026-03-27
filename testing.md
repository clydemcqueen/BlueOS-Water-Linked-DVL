# Testing Configuration

The BlueOS Water Linked DVL extension behavior can be modified for testing using three primary environment variables:

### `DVL_SIM_MODE`
- **Behavior**: When set to `true`, the extension runs using a local simulator (`simulator.py`) for both the DVL outputs and the ArduSub MAVLink interactions (such as the submarine's attitude, depth, and heartbeat).
- **Features**: The internal simulator dynamically injects simulated behavior, including aggressive simulated seafloor slopes (reaching +/- 30 degrees in both X and Y directions). It also skips standard cable-guy and network discovery processes.

### `DVL_TEST_MODE`
- **Behavior**: When set to `true`, the extension bypasses specific BlueOS hardware and integration checks (such as `cable-guy` network service checks and writing to `/data/logs/` paths).
- **Use case**: This allows the core extension software to run locally on a desktop or laptop without requiring a full BlueOS environment or a companion computer.

### `DVL_HOSTNAME`
- **Behavior**: Setting this variable overrides the standard network-based DVL discovery process.
- **Use case**: The extension will bypass automatic local network discovery and connect directly to the TCP stream of the specified hostname or IP address (e.g., `dvl.demo.waterlinked.com`). This is useful for connecting to a remote demo server or a specific physical DVL unit on your network.
