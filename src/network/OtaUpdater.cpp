#include "OtaUpdater.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Logging.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include <memory>

#include "HttpDownloader.h"
#include "esp_wifi.h"

namespace {
constexpr char latestReleaseUrl[] =
    "https://api.github.com/repos/crosspoint-reader/crosspoint-reader/releases/latest";

// Pipes HTTP response bytes directly into the OTA flash partition via Update.write().
class OtaWriteStream final : public Stream {
 public:
  OtaWriteStream(size_t& processedRef, bool& renderRef)
      : processedRef_(processedRef), renderRef_(renderRef) {}

  size_t write(uint8_t byte) override { return write(&byte, 1); }

  size_t write(const uint8_t* buf, size_t size) override {
    // Update.write takes non-const; safe since it only reads the buffer
    const size_t written = Update.write(const_cast<uint8_t*>(buf), size);
    if (written != size) writeOk_ = false;
    processedRef_ += written;
    renderRef_ = true;
    return written;
  }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }

  bool ok() const { return writeOk_; }

 private:
  size_t& processedRef_;
  bool& renderRef_;
  bool writeOk_ = true;
};
}  // namespace

OtaUpdater::OtaUpdaterError OtaUpdater::checkForUpdate() {
  // Use HttpDownloader to handle both chunked and non-chunked responses correctly
  std::string response;
  if (!HttpDownloader::fetchUrl(latestReleaseUrl, response)) {
    LOG_ERR("OTA", "Failed to fetch release info");
    return HTTP_ERROR;
  }

  JsonDocument filter;
  filter["tag_name"] = true;
  filter["assets"][0]["name"] = true;
  filter["assets"][0]["browser_download_url"] = true;
  filter["assets"][0]["size"] = true;

  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, response, DeserializationOption::Filter(filter));
  if (error) {
    LOG_ERR("OTA", "JSON parse failed: %s", error.c_str());
    return JSON_PARSE_ERROR;
  }

  if (!doc["tag_name"].is<std::string>()) {
    LOG_ERR("OTA", "No tag_name found");
    return JSON_PARSE_ERROR;
  }

  if (!doc["assets"].is<JsonArray>()) {
    LOG_ERR("OTA", "No assets found");
    return JSON_PARSE_ERROR;
  }

  latestVersion = doc["tag_name"].as<std::string>();

  for (size_t i = 0; i < doc["assets"].size(); i++) {
    if (doc["assets"][i]["name"] == "firmware.bin") {
      otaUrl = doc["assets"][i]["browser_download_url"].as<std::string>();
      otaSize = doc["assets"][i]["size"].as<size_t>();
      totalSize = otaSize;
      updateAvailable = true;
      break;
    }
  }

  if (!updateAvailable) {
    LOG_ERR("OTA", "No firmware.bin asset found");
    return NO_UPDATE;
  }

  LOG_DBG("OTA", "Found update: %s", latestVersion.c_str());
  return OK;
}

bool OtaUpdater::isUpdateNewer() const {
  if (!updateAvailable || latestVersion.empty() || latestVersion == CROSSPOINT_VERSION) {
    return false;
  }

  int currentMajor, currentMinor, currentPatch;
  int latestMajor, latestMinor, latestPatch;

  const auto currentVersion = CROSSPOINT_VERSION;

  // semantic version check (only match on 3 segments)
  sscanf(latestVersion.c_str(), "%d.%d.%d", &latestMajor, &latestMinor, &latestPatch);
  sscanf(currentVersion, "%d.%d.%d", &currentMajor, &currentMinor, &currentPatch);

  /*
   * Compare major versions.
   * If they differ, return true if latest major version greater than current major version
   * otherwise return false.
   */
  if (latestMajor != currentMajor) return latestMajor > currentMajor;

  /*
   * Compare minor versions.
   * If they differ, return true if latest minor version greater than current minor version
   * otherwise return false.
   */
  if (latestMinor != currentMinor) return latestMinor > currentMinor;

  /*
   * Check patch versions.
   */
  if (latestPatch != currentPatch) return latestPatch > currentPatch;

  // If we reach here, it means all segments are equal.
  // One final check, if we're on an RC build (contains "-rc"), we should consider the latest version as newer even if
  // the segments are equal, since RC builds are pre-release versions.
  if (strstr(currentVersion, "-rc") != nullptr) {
    return true;
  }

  return false;
}

const std::string& OtaUpdater::getLatestVersion() const { return latestVersion; }

OtaUpdater::OtaUpdaterError OtaUpdater::installUpdate() {
  if (!isUpdateNewer()) {
    return UPDATE_OLDER_ERROR;
  }

  render = false;
  esp_wifi_set_ps(WIFI_PS_NONE);

  // RAII guard: restore WiFi power saving on all exit paths
  struct PsRestorer {
    ~PsRestorer() { esp_wifi_set_ps(WIFI_PS_MIN_MODEM); }
  } psRestorer;

  std::unique_ptr<NetworkClientSecure> client(new NetworkClientSecure());
  client->setInsecure();  // Skip cert validation, same approach as HttpDownloader

  HTTPClient http;
  http.begin(*client, otaUrl.c_str());
  // GitHub release assets redirect from github.com to objects.githubusercontent.com (different host),
  // so HTTPC_FORCE_FOLLOW_REDIRECTS is required to follow cross-host redirects.
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.addHeader("User-Agent", "CrossPoint-ESP32-" CROSSPOINT_VERSION);

  const int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    LOG_ERR("OTA", "HTTP GET failed: %d", httpCode);
    http.end();
    return HTTP_ERROR;
  }

  const int64_t reported = http.getSize();
  const size_t firmwareSize = (reported > 0) ? static_cast<size_t>(reported) : UPDATE_SIZE_UNKNOWN;

  if (!Update.begin(firmwareSize)) {
    LOG_ERR("OTA", "Update.begin failed: %s", Update.errorString());
    http.end();
    return INTERNAL_UPDATE_ERROR;
  }

  OtaWriteStream otaStream(processedSize, render);
  const int written = http.writeToStream(&otaStream);
  http.end();

  if (written < 0 || !otaStream.ok()) {
    LOG_ERR("OTA", "Firmware download failed: %d", written);
    Update.abort();
    return HTTP_ERROR;
  }

  if (!Update.end(true)) {
    LOG_ERR("OTA", "Update finalization failed: %s", Update.errorString());
    return INTERNAL_UPDATE_ERROR;
  }

  LOG_INF("OTA", "Update completed");
  return OK;
}
