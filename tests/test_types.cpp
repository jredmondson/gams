#define TEST_TYPES
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 

// GAMS
#include <gams/types/Datatypes.h>

// MADARA
#include <madara/knowledge/KnowledgeBase.h>
#include <madara/knowledge/Any.h>
#include "madara/utility/SupportTest.h"
#include "madara/knowledge/CapnAdapt.h"
#include "madara/logger/GlobalLogger.h"

// CAPNP
#include "capnp/schema.h"

// C++
#include <iostream>
#include <fcntl.h>
#include <vector>

// Boost
#include <boost/filesystem.hpp>

int gams_fails = 0;

using namespace madara;
using namespace knowledge;
namespace fs = ::boost::filesystem;

KnowledgeBase k;

template<typename... Args>
inline void log(Args&&... args) {
  using namespace madara::logger;
  madara_logger_ptr_log (global_logger.get(), LOG_ALWAYS,
    std::forward<Args>(args)...);
}

#define TEST_OP(lhs, op, rhs) \
  do { \
    decltype(lhs) l = (lhs); \
    decltype(rhs) r = (rhs); \
    std::ostringstream msg; \
    msg << #lhs " [" << l << "] " #op " " #rhs " [" << r <<"] "; \
    std::string smsg = msg.str(); \
    const char *cmsg = smsg.c_str(); \
    if (l op r) { \
      log("SUCCESS : %s\n", cmsg); \
    } else { \
      log("FAIL    : %s\n", cmsg); \
      ++gams_fails; \
    } \
  } while(0)

#define TEST_EQ(lhs, rhs) TEST_OP(lhs, ==, rhs)
#define TEST_GT(lhs, rhs) TEST_OP(lhs, >,  rhs)

/*
  \brief Another sanity and demo test. Shows the user how to register types and proves that it works by registering them.
*/
void test_registry()
{
   gams::types::register_all_datatypes();
}

/*
  \brief Another sanity and demo test. Shows the user how to interface with a LaserScan type that contains a big array of data, as well as shows that a standard type that is generated is working.
*/
void test_scan()
{
   // Building the object
   ::capnp::MallocMessageBuilder scan_b;
   auto scan = scan_b.initRoot<gams::types::LaserScan>();
   
   // Setting some scan fields
   scan.setAngleMin(0);
   scan.setAngleMax(M_PI);

   // Initializing a scan data list and populating it
   auto ranges = scan.initRanges(1000);
   ranges.set(0, 1.0);
   ranges.set(999, 1.0);

   // Setting to KB
   k.set_any("scan", CapnObject<gams::types::LaserScan>(scan_b));

   // Retrieving from KB
   CapnObject<gams::types::LaserScan> k_scan = k.get("scan").to_any<CapnObject<gams::types::LaserScan> >();

   TEST_EQ(k_scan.reader().getAngleMin(), 0);
   TEST_GT((k_scan.reader().getAngleMax() - M_PI), 0);   
   TEST_EQ(k_scan.reader().getRanges()[0], 1.0);
   TEST_EQ(k_scan.reader().getRanges()[999], 1.0);      
}

/*
  \brief Another sanity and demo test. Shows the user how to interface with a CapnObject that uses many fields as well as shows that a standard type that is generated is working.
*/
void test_imu()
{
   ::capnp::MallocMessageBuilder imu_b;
   auto imu = imu_b.initRoot<gams::types::Imu>();

   auto lin_acc = imu.initLinearAccelerationCovariance(9);
   lin_acc.set(0, 0.01);
   lin_acc.set(1, 0.03);

   ::capnp::MallocMessageBuilder header_b;
   auto header = header_b.initRoot<gams::types::Header>();
   header.setStamp(10);
   header.setFrameId("world");
   header.setSeq(100);

   imu.setHeader(header);

   k.set_any("imu", CapnObject<gams::types::Imu>(imu_b));

   CapnObject<gams::types::Imu> imu_ = k.get("imu").to_any<CapnObject<gams::types::Imu> >();

   TEST_EQ(imu_.reader().getHeader().getStamp(), 10);
   TEST_EQ(imu_.reader().getHeader().getFrameId().cStr(), std::string("world"));
   TEST_EQ(imu_.reader().getHeader().getSeq(), 100);
   TEST_EQ(imu_.reader().getLinearAccelerationCovariance()[0], 0.01);
   TEST_EQ(imu_.reader().getLinearAccelerationCovariance()[1], 0.03);
}

/*
  \brief Tests that a schema in src/gams/types can be loaded at runtime and modified/used.
*/
void test_laser_schema()
{
   ::capnp::MallocMessageBuilder dyn_b;

   int fd = open(utility::expand_envs("$(GAMS_ROOT)/src/gams/types/LaserScan.capnp.bin").c_str(), 0, O_RDONLY);
   ::capnp::StreamFdMessageReader schema_message_reader(fd);
   auto schema_reader = schema_message_reader.getRoot<capnp::schema::CodeGeneratorRequest>();
   ::capnp::SchemaLoader loader;
   std::map<std::string, capnp::Schema> schemas;

   for (auto schema : schema_reader.getNodes()) {
    schemas[schema.getDisplayName()] = loader.load(schema);
   }

   auto schema = schemas.at("src/gams/types/LaserScan.capnp:LaserScan").asStruct();
   auto dynbuilder = dyn_b.initRoot<capnp::DynamicStruct>(schema);

   ::capnp::MallocMessageBuilder header_b;
   auto header = header_b.initRoot<gams::types::Header>();

   header.setStamp(10);
   header.setFrameId("world");
   header.setSeq(100);

   dynbuilder.set("angleMin", 0.1);

   //k.set_any("dynamic_laser", dynbuilder);

   // Could not find dox on how to do this.
   //dynbuilder.set("header", header);

}

/*
  \brief Tests that a schema in src/gams/types can be loaded at runtime and modified/used.
*/
bool test_schema(const fs::path& path)
{

   log("Loading schema %s\n", path.c_str());
   ::capnp::MallocMessageBuilder dyn_b;

   int fd = open(path.c_str(), 0, O_RDONLY);
   log("Schema opened %s\n", path.c_str());   
   ::capnp::StreamFdMessageReader schema_message_reader(fd);

   auto schema_reader = schema_message_reader.getRoot<capnp::schema::CodeGeneratorRequest>();
   ::capnp::SchemaLoader loader;
   std::map<std::string, capnp::Schema> schemas;

   for (auto schema : schema_reader.getNodes()) {
    schemas[schema.getDisplayName()] = loader.load(schema);
   }

   log("Building dynamic struct for %s", path.c_str());
   auto schema = schemas.at(path.filename().c_str()).asStruct();
   auto dynbuilder = dyn_b.initRoot<capnp::DynamicStruct>(schema);

   close(fd);

   return true;
   //k.set_any("dynamic_laser", dynbuilder);

   // Could not find dox on how to do this.
   //dynbuilder.set("header", header);
}



/*
  \brief Tests that all of the schemas inside src/gams/types can be loaded at runtime.
*/
void test_all_schemas()
{
    fs::path root(utility::expand_envs("$(GAMS_ROOT)/src/gams/types").c_str());

    if(!fs::exists(root) || !fs::is_directory(root)) 
    {
       log("src/gams/types does not exist!");
       gams_fails++;
       return;
    }

    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ".capnp.bin") 
        {
            TEST_EQ(test_schema(it->path()), 1);
        } 
        ++it;
    }
}

int main (int, char **)
{
	std::cout << "Testing types" << std::endl;
	test_registry();

	test_scan();
   test_imu();
   test_laser_schema();
   test_all_schemas();

   if (gams_fails > 0)
   {
     std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
   }
   else
   {
     std::cerr << "OVERALL: SUCCESS.\n";
   }

   return gams_fails;
}
