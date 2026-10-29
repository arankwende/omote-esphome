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
homeassistant/
  omote_tv_now_playing.yaml  Home Assistant package: TV titles and posters (see below)
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

## TV now playing (Netflix, Disney+, Jellyfin...)

While the TV plays or is paused, the TV page shows the poster full screen with the title at the bottom; tap it for the controls. On the charger, the remote stays on showing the poster.

The Android TV Remote integration only reports which app is open, not what it plays. `homeassistant/omote_tv_now_playing.yaml` is a Home Assistant package that fills the gap and builds `sensor.omote_tv_now_playing`:

- **Jellyfin**: title, series and artwork from the Jellyfin integration's player for the TV.
- **Netflix, Disney+, Prime and other apps** (and Jellyfin without the integration): every 10 s while the TV is on, the Android Debug Bridge integration reads what the app on screen publishes as playing (`dumpsys media_session`). The poster is looked up by title on TMDB and served by Home Assistant, so the remote downloads it over local http.

Point the remote at it in your device file:

```yaml
substitutions:
  tv_now_playing: sensor.omote_tv_now_playing
```

Without it, `tv_now_playing` defaults to `tv_media_player`, and the page shows a poster only if that player reports a picture.

### Installing the package in Home Assistant

HACS can't install it: HACS handles integrations, cards, themes and scripts, not YAML packages. A blueprint won't do either, since blueprints only make automations and this also creates helpers, a `rest_command` and template entities. Copy it by hand once:

1. **TV**: Settings > Device preferences > About, click *Build* 7 times, then turn on *USB debugging* (and *Network debugging* if listed) in Developer options. Give the TV a fixed IP.
2. **ADB integration**: add *Android Debug Bridge* for the TV's IP and accept the prompt on the TV ("always allow"). It can live next to the Android TV Remote integration.
3. **Check that the TV publishes titles**: with a Netflix episode playing, run *Android Debug Bridge: ADB command* with `dumpsys media_session` from Developer tools > Actions. The ADB entity's `adb_response` attribute must contain a `description=<title>, ...` line; without it only the app name can be shown.
4. **TMDB token**: create a free TMDB account, copy the *API Read Access Token* (Settings > API) and add it to `config/secrets.yaml`:
   ```yaml
   tmdb_bearer: "Bearer eyJhbGciOi..."
   ```
5. **Entity ids**: Home Assistant YAML has no substitutions, so the entities are written in the file. Search and replace `media_player.living_tv_philips_oled_adb` (ADB player) and `media_player.living_tv_philips_oled_jellyfin` (Jellyfin player; leave it if you don't use the integration).
6. **Copy the file** to `config/packages/` (File editor or Studio Code Server add-on, or the Samba share). Enable packages in `configuration.yaml` if you haven't already:
   ```yaml
   homeassistant:
     packages: !include_dir_named packages
   ```
7. **Restart**: Developer tools > YAML > Check configuration, then restart. You should now have `sensor.omote_tv_now_playing`, `image.omote_tv_poster` and the `input_text.omote_tv_*` helpers.

For a second TV, copy the file, replace the two entity ids and rename the `omote_tv_*` ids so both copies can coexist.

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
