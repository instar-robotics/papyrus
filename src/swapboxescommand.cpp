/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#include "swapboxescommand.h"

#include <QDebug>

SwapBoxesCommand::SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox, QUndoCommand *parent)
  : QUndoCommand(parent),
    m_scene(scene),
    m_toSwap(toSwap),
    m_newBox(newBox)
{
	if (scene == nullptr) {
		qWarning() << "[SwapBoxesCommand] created with null scene!";
		return;
	}

	if (toSwap == nullptr) {
		qWarning() << "[SwapBoxesCommand] created with null box to swap!";
		return;
	}

	if (newBox == nullptr) {
		qWarning() << "[SwapBoxesCommand] created with null new box!";
		return;
	}
}

void SwapBoxesCommand::undo()
{
	if (m_scene == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot undo: no scene!";
		return;
	}

	if (m_toSwap == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot undo: no swap box!";
		return;
	}

	if (m_newBox == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot undo: no new box!";
		return;
	}

	// Add the m_toSwap box back on the scene
	m_scene->addBox(m_toSwap, m_toSwap->scenePos());
	m_toSwap->setSwapCandidate(false);

	// Restore all links that were FROM m_toSwap
	foreach (Link *outLink, m_outputLinks) {
		if (outLink == nullptr) {
			qWarning() << "[SwapBoxesCommand::undo] found null pointer in list of output links!";
			continue;
		}

		m_newBox->outputSlot()->removeOutput(outLink);
		m_toSwap->outputSlot()->addOutput(outLink);
		outLink->setFrom(m_toSwap->outputSlot());
	}

	// Do not forget to empty the list of linsk that were from m_toSwap
	m_outputLinks.clear();

	// Restore all connections TO the m_toSwap, by input slots
	foreach (QString inputName, m_inputLinks.keys()) {
		QList<Link *> links = m_inputLinks[inputName];

		// Look for the input slot
		foreach(InputSlot *iSlot, m_toSwap->inputSlots()) {
			if (iSlot == nullptr) {
				qWarning() << "[SwapBoxesCommand::undo] found null pointer in list of input slots!";
				continue;
			}

			if (iSlot->name() != inputName)
				continue;

			// Add all the links to this input slot
			foreach (Link *link, links) {
				// Unlink if from new input slot (might be a no-op if this link was removed)
				if (link->to() != nullptr)
					link->to()->removeInput(link);

				// Add the link back as an output from its originating box (no-op for re-linked links)
				link->from()->addOutput(link);

				iSlot->addInput(link, true);
				link->setTo(iSlot);

				// If the link was removed, add it back to the scene
				if (link->scene() == nullptr) {
					m_scene->addItem(link);
					link->addLinesToScene();
				}
			}

			break;
		}
	}

	// Do not forget to empty the list of saved connections
	m_inputLinks.clear();

	// Remove the new box from the scene (but don't delete it: we want to be able to redo)
	m_scene->removeItem(m_newBox);
	m_toSwap->update();
}

void SwapBoxesCommand::redo()
{
	if (m_scene == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot redo: no scene!";
		return;
	}

	if (m_toSwap == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot redo: no swap box!";
		return;
	}

	if (m_newBox == nullptr) {
		qWarning() << "[SwapBoxesCommand] cannot redo: no new box!";
		return;
	}

	// Add the new box to the scene
	m_scene->addBox(m_newBox, m_toSwap->scenePos());

	// Substitute the box information
	m_newBox->setUuid(m_toSwap->uuid());
	m_newBox->setSaveActivity(m_toSwap->saveActivity());
	m_newBox->setPublish(m_toSwap->publish());
	m_newBox->setTopic(m_toSwap->topic());
	m_newBox->setTitle(m_toSwap->title());
	// Rows and cols are copied (valid for matrix, and ignored for scalar)
	m_newBox->setRows(m_toSwap->rows());
	m_newBox->setCols(m_toSwap->cols());

	// Before making changes to the links, we save the state of all links: i.e. for each slots
	// (output and inputs), we keep a list of pointers to all links.

	// Save all links that are FROM m_toSwap
	foreach (Link *outLink, m_toSwap->outputSlot()->outputs()) {
		if (outLink == nullptr) {
			qWarning() << "[SwapBoxesCommand::redo] found null pointer in list of output links!";
			continue;
		}

		m_outputLinks.push_back(outLink);
	}

	// Save all links that are TO m_toSwap (by input slots)
	foreach (InputSlot *iSlot, m_toSwap->inputSlots()) {
		if (iSlot == nullptr) {
			qWarning() << "[SwapBoxes::redo] found null pointer in list of input slots!";
			continue;
		}

		QList<Link *> links;

		foreach (Link *link, iSlot->inputs()) {
			if (link == nullptr) {
				qWarning() << "[SwapBoxes::redo] found null pointer in list of inputs!";
				continue;
			}

			links.push_back(link);
		}

		m_inputLinks.insert(iSlot->name(), links);
	}

	// Now we transfer as many links as possible from the "old box" (toSwap)
	// to the new one. The idea is to keep all links from the inputs who have the
	// same name, and discard the others.
	// Be warned, though: same name doesn't necessarily imply same type, so the
	// copied links might be invalid.
	foreach (InputSlot *swapSlot, m_toSwap->inputSlots()) {
		bool matchFound = false;

		if (swapSlot == nullptr) {
			qWarning() << "Null pointer found in the list of input slots of the box to swap!";
			continue;
		}

		// For each input of the box to swap, look in the new box's inputs for similarly-named
		// This is N*M, which is ugly :/
		foreach (InputSlot *iSlot, m_newBox->inputSlots()) {
			if (iSlot == nullptr) {
				qWarning() << "Null pointer found in the new box.";
				continue;
			}

			if (swapSlot->name() != iSlot->name())
				continue;

			matchFound = true;

			// We found an input slot named similarly, so now, transfer all links
			foreach (Link *link, swapSlot->inputs()) {
				// First, if this is a self-link, update the origin
				if (link->from()->box() == m_toSwap) {
					m_toSwap->outputSlot()->removeOutput(link);
					link->setFrom(m_newBox->outputSlot());
					m_newBox->outputSlot()->addOutput(link);
				}

				// Now update the destination box (should be done after, because
				// 'setTo()' checks origin box to set 'isSelfLoop'
				link->setTo(iSlot);

				// Remove this link from the inputs of the box to swap
				swapSlot->removeInput(link);

				// Add this link as inputs for the new box
				iSlot->addInput(link, true);

				// Check if this new link is invalid
				if (link->checkIfInvalid() && m_scene != nullptr && m_scene->script() != nullptr)
					m_scene->script()->setIsInvalid(true);
			} // end of foreach on m_swapSlot->inputs()
		} // end of foreach on m_toSwap->inputSlotS()

		// If an input slot (of the box to swap) was not found in the new box, remove all its
		// connected links from the scene
		if (!matchFound) {
			foreach (Link *link, swapSlot->inputs()) {
				swapSlot->removeInput(link);
				link->from()->removeOutput(link);
				// NOTE: we do NOT call setTo(nullptr) nor setFrom(nullptr) because then the
				// information is lost and it's not possible to undo()
				link->removeLinesFromScene(); // very important to call (otherwise it segfaults)
				m_scene->removeItem(link);
				m_scene->update();
			}
		}
	} // end of foreach on m_newBox->inputSlots()

	// Also re-link all outputs
	foreach (Link *link, m_toSwap->outputSlot()->outputs()) {
		if (link == nullptr) {
			qWarning() << "Null pointer found in output slot of a box to swap";
			continue;
		}

		m_toSwap->outputSlot()->removeOutput(link);
		m_newBox->outputSlot()->addOutput(link);
		link->setFrom(m_newBox->outputSlot());

		// Check if this new link is invalid
		if (link->checkIfInvalid() && m_scene != nullptr && m_scene->script() != nullptr)
			m_scene->script()->setIsInvalid(true);

	} // end of foreach on m_toSwap->outputSlot()->outputs()

	// Check if the new box is invalid
	if (m_newBox->checkIfBoxInvalid() && m_scene != nullptr && m_scene->script() != nullptr)
		m_scene->script()->setIsInvalid(true);

	// Then remove the swap box (do NOT delete item as we might still want to undo)
	m_scene->removeItem(m_toSwap);
	// TODO: emit message
}
