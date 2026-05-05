#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "octree_storage.h"

namespace fs = std::filesystem;
using namespace sonar_3d_reconstruction;

namespace {

// Build a complete valid IWLO file with N entries, then truncate to 'truncate_at' bytes.
// Returns the temp directory path.
std::string make_truncated_meta(uint64_t declared_count,
                                uint64_t actual_entries,
                                size_t extra_truncate_bytes = 0) {
    auto tmpdir = fs::temp_directory_path() / fs::path("phase_c_octree_load_XXXX");
    fs::create_directories(tmpdir);
    std::string meta_path = (tmpdir / "iwlo_meta.bin").string();

    std::ofstream ofs(meta_path, std::ios::binary);
    const uint32_t magic = 0x49574C4F;
    const uint32_t version = 1;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&declared_count), sizeof(declared_count));

    for (uint64_t i = 0; i < actual_entries; ++i) {
        unsigned short k0 = static_cast<unsigned short>(i);
        unsigned short k1 = 0;
        unsigned short k2 = 0;
        double log_odds = 0.5;
        int obs = 1;
        ofs.write(reinterpret_cast<const char*>(&k0), sizeof(k0));
        ofs.write(reinterpret_cast<const char*>(&k1), sizeof(k1));
        ofs.write(reinterpret_cast<const char*>(&k2), sizeof(k2));
        ofs.write(reinterpret_cast<const char*>(&log_odds), sizeof(log_odds));
        ofs.write(reinterpret_cast<const char*>(&obs), sizeof(obs));
    }
    ofs.close();

    if (extra_truncate_bytes > 0) {
        size_t cur = fs::file_size(meta_path);
        if (cur > extra_truncate_bytes) {
            fs::resize_file(meta_path, cur - extra_truncate_bytes);
        }
    }

    // Do NOT create octree.bt — load() skips it when absent (calls octree_->clear()).

    return tmpdir.string();
}

}  // namespace

TEST(OctreeStorageLoad, ReturnsFalseOnTruncatedMeta) {
    // Declare 3 entries but write only 2 -> third read should fail.
    std::string dir = make_truncated_meta(3, 2);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_FALSE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 0u)
        << "Storage must be cleared on partial-read failure";
    fs::remove_all(dir);
}

TEST(OctreeStorageLoad, ReturnsFalseOnMidEntryTruncation) {
    // Declare 2 entries, write 2, then chop off 4 bytes from the very end
    // (mid second-entry's int observation_count). Last read fails partway.
    std::string dir = make_truncated_meta(2, 2, /*extra_truncate=*/4);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_FALSE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 0u);
    fs::remove_all(dir);
}

TEST(OctreeStorageLoad, ReturnsTrueOnCompleteMeta) {
    // Declare 4 entries, write 4 -> normal round-trip.
    std::string dir = make_truncated_meta(4, 4);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_TRUE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 4u);
    fs::remove_all(dir);
}
