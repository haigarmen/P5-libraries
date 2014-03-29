/*
 * Gestalt
 *
 * Copyright (C) 2012 Patrick Kochlik + Dennis Paul
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * {@link http://www.gnu.org/licenses/lgpl.html}
 *
 */


package gestalt.extension.picking;


import static gestalt.Gestalt.PICKING_BIN_2D;
import static gestalt.Gestalt.PICKING_BIN_3D;
import gestalt.shape.AbstractDrawable;


public abstract class Picker
    extends AbstractDrawable {

    public PickingBin[] bin;

    private boolean isActive;

    public Picker() {
        setupPickingbins();
        isActive = true;
    }


    private void setupPickingbins() {
        final int myShapeBinSize = 10;
        bin = new PickingBin[2];

        PickingBin mySpatialTransparentShapeBin = new PickingBin(myShapeBinSize);
        bin[PICKING_BIN_3D] = mySpatialTransparentShapeBin;

        PickingBin myOrthoForegroundShapeBin = new PickingBin(myShapeBinSize);
        bin[PICKING_BIN_2D] = myOrthoForegroundShapeBin;
    }


    public boolean isActive() {
        return isActive;
    }


    public void setActive(boolean theIsActive) {
        isActive = theIsActive;
    }
}
