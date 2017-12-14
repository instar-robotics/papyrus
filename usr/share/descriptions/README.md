# Description Directory

This file is temporary (for now) and should be replaced by a directory that is
globally accessible, most likely something in `/usr/share/papyrus` or equivalent.

It should contain the `.xml` files containing the description of the neural "boxes".

The neural boxes should be groupes into categories, these categories should be
directories inside the present directory.

This is automatically parsed by Papyrus at startup (and hopefully refreshed?) so
that in order to create a new category, it is enough to create a new directory.

A neural box is comprised of a `.xml` file whose template can be found in this
(temp) directory and an `.svg` icon that should have the same name.
