#ifndef CHANGELOG_H
#define CHANGELOG_H

#include <QString>

QString changelog = "<h3>CHANGELOG</h3>"
                    "<ul>"

                    "<li><strong>v0.5.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>When saving the same script, without adding any boxes, and just making "
                    "position changes, or changing the weight of a link, the order in which "
                    "functions and links were written in the XML was not deterministic. It has been "
                    "fixed, and now both functions and links are written in order of their UUID. "
                    "Moving boxes or changing link weights will produce only a small git diff for "
                    "the coordinates / weights.</li>"
                    "<li>"
                    "<li>There was a segfault when closing Papyrus after opening two scripts. This "
                    "prevented the normal cleaning procedures to happen and thus settings to be "
                    "saved (among other things). This is now fixed.</li>"
                    "<li>The libraries in the library panel on the left are not sorted "
                    "alphabetically (they were sorted in the reverse order)</li>"
                    "<li>Lots of XML information about the function boxes were saved in the script's"
                    " XML and thus any changed in alexandria (such as the icon, the name of inputs, "
                    "etc.) were not being reloaded. Now only the minimum information is stored in "
                    "the XML, instead, much of the information regarding a box are being re-parsed "
                    "from the library when a script is opened.</li>"
                    "<li>There was a compilation error on older Wt version, because a QStringRef "
                    "could not be used in the same manner, this was converted back to QString to fix"
                    " this.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.5.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Parse <description> tags from XML description files and display a "
                    "function's description (and its input) when hovering the library</li>"
                    "<li>Fix the graphical glitch where a link seemed to \"go out in the distance\" "
                    "when a Comment Zone was resized and the box was now out of the zone</li>"
                    "<li>The full version MAJOR.MINOR.BUGFIX is now displayed in the Home Page and "
                    "the About dialog (the bugfix version was missing) which allows the user to know"
                    " which version he is running</li>"
                    "<li>Only one Comment Zone was parsed & loaded when a script was opened, this is"
                    " now fixed</li>"
                    "<li>Function boxes used to be transparent which made them weird-looking when "
                    "placed inside Comment Zones, now their background is white</li>"
                    "<li><strong>(new feature)</strong> Papyrus now periodically checks the gitlab repository for a "
                    "new release and when it finds one, it warns the user with a dialog</li>"
                    "<li><strong>(new feature)</strong> Papyrus will now re-open with the last "
                    "opened scripts (and the last active script) by default. It is possible, however"
                    " to disable this feature in the options</li>"
                    "<li><strong>(new feature)</strong> The full CHANGELOG is now visible in the "
                    "Help > CHANGELOG menu. Also, when it's the first time a new version is "
                    "launched, the CHANGELOG is automatically displayed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the big issue when events on properties panel (such as clicking OK, CANCEL, "
                    "or pressing ENTER) was propagated to all open scenes. It was thus unsafe to edit "
                    "several scripts at the same time. Now it should be safe.</li>"
                    "<li>Auto scaling features for the bar and graph visualization was not symmetric, so "
                    "it was difficult to read where the 0 lien was. Now it is symmetric, the 0 line is "
                    "thus always centered</li>"
                    "<li>When visualizing several neurons, the bars colors were cycled, and when no "
                    "more colors were found, it would switch to alpha channel, which made it "
                    "difficult to read. Now, all neurons have the same color</li>"
                    "<li>Papyrus segfaulted when a new script was created or opened and the ROS "
                    "master was down. This is now fixed.</li>"
                    "<li>It was not possible to save the scripts which were invalid. This was "
                    "problematic when you had to save your work and go, and did not have the time to"
                    " fix the problem. Now there's still a warning message, but saving is still"
                    " performed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Papyrus used to segfault when naming or renaming a script with a space in "
                    "the name, this was due to ROS not accepting spaces in topic names</li>"
                    "<li>The multiple XML argument for inputs is now optional and will default to "
                    "false when not specified. It is now encouraged not to add multiple = \"false\" "
                    "to make XML files more lightweight.</li>"
                    "<li>Papyrus did not check if an input was multiple or not before creating Link."
                    " Now it does, so it's not possible to connect several functions to an input "
                    "that is marked multiple</li>"
                    "<li>There was a problem saving the secondary argument: it used to be that Links"
                    " were saved as secondary if and only if it was a link from the same box it "
                    "started. Now it checks the real value.</li>"
                    "<li>When saving a script for the first time, it will use the script's name as "
                    "the default value for the XML file. So that the user doesn't have to manually "
                    "type it. Note: the script name is sanitized, so it means it is safe to use "
                    "spaces in script name: Visual Docking will be transformed as VisualDocking.xml."
                    " So please use spaces if you want to.</li>"
                    "<li>When opening a script, the state of the interface buttons (play, pause, "
                    "stop) was not initiated, thus was actually random. Now they are initiated to "
                    "the state where the script had not been launched (thus \"play\" visible only).</li>"
                    "<li>When a script was saved, there one warning message per constant function "
                    "box talking about missing description file. This is now fixed (built-in "
                    "Constants do not have description files)</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Implement a reliable, asynchronous communication with Kheops nodes.<br>"
                    "Now, starting, pausing and stopping a node is reliable (either when creating "
                    "and launching a node, or connecting to an existing node). The interface is "
                    "reliable and updated to the node's reported status.<br>"
                    "Basically, if you see a play button but no stop button, it means the script is "
                    "not launched. Is you see a play and a stop button, it means the script is "
                    "launched, but paused (clicking \"play\" will resume the execution).<br>"
                    "If you see a pause button, it means the script is launched and running "
                    "(clicking the pause button will pause the script's execution).>/li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.3.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Autosave feature is now implemented: when working on a script, it is "
                    "autosaved every 60 seconds (in a .autosave file). When one tries to open a "
                    "script, Papyrus will detect if there is an autosaved version of this file and "
                    "offer to open it instead (one can accept or refuse). Autosaved files are "
                    "destroyed when the script file is saved</li>"
                    "<li>When a script was opened, there was a bug where the script just before it "
                    "in the tab list would take its name. Thus we would have two tabs with the same "
                    "name. This is now fixed</li>"
                    "<li>The input and output slots of functions are now colorized according to "
                    "their types. At the moment, STRING-based slots are colorized cyan, SCALAR-based"
                    " types are left white and MATRIX-based slots are colorized pink. This makes it "
                    "much easier to see the compatible inputs</li>"
                    "<li>Function boxes can now have individual names. By default they don't have a "
                    "name and they function name is displayed. To give a name, edit the field in the"
                    " properties panel. When a function box has a name, it is displayed instead of "
                    "its function name. To remove a name (and restore displaying the function's "
                    "name), simply delete the name and validate.</li>"
                    "<li>Modifications in the properties panel can now be validated by pressing Enter "
                    "and cancelled by pressing ESCAPE (before this, one had to click on the Ok or "
                    "Cancel buttons ; now both choices are available).</li>"
                    "<li>Changes in the properties panel are now reflected on the scene as soon as "
                    "Enter is pressed (or Ok is clicked). Previous to this, it was necessary to "
                    "click on the scene with the mouse to trigger a repaint. This is no longer "
                    "necessary.</li>"
                    "<li>When changing some parameters in the properties panel, the script is now "
                    "set as \"modified\".</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.2.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the icons for the ConstantBox that were going outside the box (this bug"
                    " happens when boxes were resized from 3 squares to 2). Now their icons are "
                    "displayed properly</li>"
                    "<li>I detected that if there was an issue (such as a segfault) during the save "
                    "operation, not only would the save operation fail, but the script file would be"
                    " emptied!<br>"
                    "This means it was possible to potentially lose a lot a work by trying to save.<br>"
                    "Now this is fixed: the saving is performed in a temporary file, and only when "
                    "this temporary file is written and successfully closed, it is copied on top of "
                    "the original script file.<br>"
                    "In case there's a segfault or some other error, only the modifications are lost"
                    " but the original script file is left untouched.</li>"
                    "</ul>"
                    "</li>"


                    "</ul>";

#endif // CHANGELOG_H
