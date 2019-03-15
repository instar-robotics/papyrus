#ifndef CONSTANTS
#define CONSTANTS

/**
 * A set of project-wide constants
 */

// Define the name of the application
#define APP_NAME "Papyrus"

// Define the scroll factor for the QGraphicsView
#define SCALE_FACTOR 1.2

// Define the prefix in which to search for ressources
// TEMPORARY: should be replaced by a global 'settings' file for Papyrus
#define RESOURCE_DIR "/home/nschoe/workspace/Qt/papyrus/usr/share/"
// #define RESOURCE_DIR "/usr/share/"

// Define the size of the icons in the left 'Library' pane (in px)
#define LIBRARY_ICON_SIZE 40

// Define the default name for a new script
#define NEW_SCRIPT_DEFAULT_NAME "Untitled"

// Define the margin added to the scene's rectangle when resizing it (in px)
#define SCENE_RECT_MARGIN 200

// Define the name of the root element XML tag in the function's description files
#define XML_ROOT_ELEM "description"

// Major version number
#define MAJOR_VERSION 0

// Minor version number
#define MINOR_VERSION 5

// Bugfix version number
#define BUGFIX_VERSION 1

// Define the maximum number of rows / columns a matrix can have (for the spinbox)
#define MAX_ROWS 100000
#define MAX_COLS 100000

// Define the minimum and maximum allowed weight (for the double spinbox)
#define MIN_WEIGHT -10000000
#define MAX_WEIGHT 10000000

// Define the number of decimals for the double spinbox for link's weight
#define LINKS_NB_DECIMALS 8

// Define the minimum and maximum time value (for the double spinbox)
#define MIN_TIME_VALUE 0 //0.001
#define MAX_TIME_VALUE 10000

// Define the z-value for the links (used to put them behind slots to prevent hiding the slots)
#define LINKS_Z_VALUE -1.0

// Define the z-value for the data visualization windows (above boxes and links)
#define DATA_Z_VALUE 10

// Define the z-value for the rectangular comments (under everything)
#define COMMENTS_Z_VALUE -10.0

// Define the z-value for the neural boxes
#define BOXES_Z_VALUE 5

// Define the time (in minutes) after which the user is notified about modified, unsaved scripts
#define TIME_WARN_MODIFIED 10

// Define the organisation name, and domain for use with QSettings
#define ORGA "INSTAR Robotics"
#define DOMAIN "instar-robotics.com"

// Define the period for the autosave script
#define AUTOSAVE_PERIOD 60000 // 1 minute

// Time (in ms) during which status bar messages are displayed
#define MSG_DURATION 3000

#endif // CONSTANTS

