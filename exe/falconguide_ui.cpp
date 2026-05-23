#include "falconguide/ui/ui_app.hpp"

#include "falconguide/logger/logger.hpp"

#include <cstdlib>

int main() {
    falconguide::log::Init();

    falconguide::ui::UiApp app;
    app.Run();

    falconguide::log::Shutdown();
    return EXIT_SUCCESS;
}
