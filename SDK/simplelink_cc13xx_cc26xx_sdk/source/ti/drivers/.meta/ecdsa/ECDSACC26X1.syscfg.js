/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *  ======== ECDSACC26X1.syscfg.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let trngIntPriority = Common.newIntPri()[0];
trngIntPriority.name = "trngInterruptPriority";
trngIntPriority.displayName = "TRNG Interrupt Priority";
trngIntPriority.description = "TRNG interrupt priority used to generate the PMSN.";

/*
 *  ======== getLibs ========
 *  Argument to the /ti/utils/build/GenLibs.cmd.xdt template
 */
function getLibs(mod)
{
    /* Get device information from GenLibs */
    let GenLibs = system.getScript("/ti/utils/build/GenLibs");

    let libGroup = {
        name: "/third_party/ecc",
        deps: [],
        libs: [GenLibs.libPath("third_party/ecc", "ecc_cc13x1_cc26x1.a")],
        allowDuplicates: true
    };

    return (libGroup);
}

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base ECDSA configuration
 */
let devSpecific = {
    config: [
        trngIntPriority
    ],

    templates : {
        boardc: "/ti/drivers/ecdsa/ECDSACC26X1.Board.c.xdt",
        boardh: "/ti/drivers/ecdsa/ECDSA.Board.h.xdt",

        /* contribute libraries to linker command file */
        "/ti/utils/build/GenLibs.cmd.xdt":
            {modName: "/ti/drivers/ECDSA", getLibs: getLibs}
    }
};

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "ECDSA", null,
        [{name: "ECDSACC26X1"}], null);

    /* merge and overwrite base module attributes */
    return (Object.assign({}, base, devSpecific));
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by base ECDH module */
    extend: extend
};
