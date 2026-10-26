# omote-esphome
Omote Esphome Yaml with Music Assistant Control
I used LLM's heavily in designing these yamls (started manually then took advantage of them).

This project is inspired by https://github.com/tobsch/nanopod, https://github.com/Zebble/OMOTE-ESPHome and https://github.com/Wolverine-14/Omote-ESPHome all great implementations in ESPHome.

Currently, the following are working:
1. LVGL display, including in the omote yaml 3 pages (one for a TV, one for Music Assistant and one for a room with HA)
2. Button matrix using a custom esphome component (https://github.com/arankwende/tca8418-keypad-esphome), all buttons work.
3. Accelerometer using a custom esphome componente (https://github.com/arankwende/lis3dh-esphome), including wake up.
4. Battery gauge sensor
5. IR Receiver
6. IR transmitter


## File layout

```
omote.yaml                    <- firmware entry point (Home Assistant music)
omote-music-assistant.yaml    <- same remote, music via Music Assistant (builds on omote.yaml)
packages/
  defaults.yaml    default value of every setting (override in your device file)
  core.yaml        device, WiFi, API, OTA, time
  hardware.yaml    OMOTE Rev5 board: display, touch, keypad chip, accelerometer, IR
  power.yaml       battery, dim / screen-off / deep sleep, wake handling
  keypad.yaml      physical keys: actions, on-screen feedback, hold-to-repeat
  ui.yaml          LVGL theme, status bar, toast, page navigation, fonts
  room.yaml        Room page (climate, light, blinds)
  music.yaml       Music page via Home Assistant
  tv.yaml          TV page (IR, TV media player, apps)
  settings.yaml    Settings page + adjustable settings
  music-assistant.yaml  Music Assistant overrides for music.yaml (+ playlist picker, queue)
  speakers-ma.yaml Speakers page: control and group up to 6 Music Assistant players
```

Don't edit the files in `packages/`: put your values in a device file (see `examples/`) that overrides `packages/defaults.yaml`. Pages appear in the order of the `packages:` list.

### Using it from the ESPHome dashboard

Copy `omote.yaml`, `omote-music-assistant.yaml` and the `packages/` folder into your ESPHome config folder (`/config/esphome/` in Home Assistant). The dashboard lists only the top-level YAML files as devices; the files in `packages/` are pulled in by `!include`. Alternatively, keep a small device file on the server and pull the rest from GitHub:

```yaml
substitutions:
  name: omote
  # ...your overrides of the values in omote.yaml
packages:
  omote:
    url: https://github.com/arankwende/omote-esphome
    ref: main          # better: pin a tag
    files: [omote.yaml]
    refresh: 1d
```

## Power management

`packages/power.yaml` follows the same sleep scheme as the original OMOTE Rev5 firmware:

1. After `screen_dim_timeout` seconds without activity (touch, any key, or motion) the screen dims; at night it turns off instead.
2. After `screen_off_timeout` the backlight and keypad backlight turn off and LVGL pauses.
3. After `sleep_timeout` the remote enters deep sleep. Before sleeping it parks and holds the LCD, backlight, SD, IR and keypad-backlight pins so nothing draws current.
4. Wake-up comes from any key (TCA8418 INT, GPIO8) or from motion (LIS3DH INT1, GPIO2). Both are active-low and share one `esp32_ext1_wakeup` with `ANY_LOW`.

The power key sleeps immediately with **keypad-only** wake, so the hand holding the remote doesn't wake it again.

The **Stay Awake** switch in Home Assistant disables deep sleep, for logs or OTA. It turns on by itself when an OTA starts and resets on reboot. Since the remote sleeps, do OTA updates right after waking it, or turn on Stay Awake first.

## Music Assistant variant

`omote-music-assistant.yaml` (with `packages/music-assistant.yaml`) is the same remote, but all music control goes **directly to the Music Assistant server** (its `/api` endpoint) instead of through Home Assistant `media_player` entities. That covers transport, volume, mute, quick-play playlists, now-playing info and artwork, and joining speakers into a group. It includes `omote.yaml`, so every other page and the power management stay shared.

Setup: add `ma_token` (an MA long-lived token) to `secrets.yaml`, set `ma_url`, `ma_player_id` and up to four `ma_speaker_N_id`/`_name` entries in the file's `substitutions`, then flash `omote-music-assistant.yaml`. MA player ids are not HA entity ids; look them up with `players/all` at `http://<ma-host>:8095/api-docs`.

## Secrets

`omote.yaml` reads its keys from `secrets.yaml`:

```yaml
wifi_ssid: "..."
wifi_password: "..."
omote_api_key: "..."        # 32-byte base64 API encryption key (the old ha_api_key value)
ma_token: "..."             # only for omote-music-assistant.yaml
```

OTA uploads are encrypted with the same `omote_api_key`; there is no separate OTA password.

## Settings

The Settings page (fourth page) and the matching Home Assistant entities control screen brightness, sleep timeout, lift to wake and its sensitivity. It also shows battery, the estimated runtime and WiFi signal. Settings survive deep sleep.
