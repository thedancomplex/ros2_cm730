#include "cm730driver/cm730driver.hpp"

#include "cm730driver/cm730device.hpp"
#include "cm730driver/pingservice.hpp"
#include "cm730driver/readservice.hpp"
#include "cm730driver/writeservice.hpp"
#include "cm730driver/bulkreadservice.hpp"
#include "cm730driver/syncwriteservice.hpp"

#include <numeric>

using namespace std::literals::chrono_literals;

namespace cm730driver
{

Cm730Driver::Cm730Driver()
: rclcpp::Node{"cm730driver"}
{
  mDevice = std::make_shared<Cm730Device>("/dev/ttyUSB0");
  mDevice->open();

  mPingServer = PingService::create(*this, "/cm730/ping", mDevice, get_clock());
  mReadServer = ReadService::create(*this, "/cm730/read", mDevice, get_clock());
  mWriteServer = WriteService::create(*this, "/cm730/write", mDevice, get_clock());
  mBulkReadServer = BulkReadService::create(*this, "/cm730/bulkread", mDevice, get_clock());
  mSyncWriteServer = SyncWriteService::create(*this, "/cm730/syncwrite", mDevice, get_clock());
}

Cm730Driver::~Cm730Driver()
{
  mDevice->close();
}

}  // namespace cm730driver
