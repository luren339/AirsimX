#include <filesystem>

int main()
{
    std::filesystem::directory_iterator entry(".");
    const std::filesystem::directory_iterator end;
    for (; entry != end; ++entry) {
        (void)entry->path();
    }
    return 0;
}
