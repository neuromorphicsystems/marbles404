/*************************************************************************
 * SCAMP Vision Chip Development System Library
 *------------------------------------------------------------------------
 * Copyright (c) 2020 The University of Manchester. All Rights Reserved.
 *
 *************************************************************************/

#include <scamp5.hpp>
#include "MISC_FUNCS.hpp"

using namespace SCAMP5_PE;

// ------------------------------------------------------------
// Register usage
// ------------------------------------------------------------

#define BOARD_REG       S5    // board mask
#define MAP_REG         S3    // static black walls / holes / track features
#define DARK_NOW_REG    S2    // current dark pixels inside board
#define BALL_CAND_REG   S0    // dynamic dark pixels: DARK_NOW & ~MAP
#define TMP_REG         S1    // temporary
// S4 free for debug if needed
// S6 temporary
// A = image
// F = display/debug analogue image

uint8_t window_size = 2;

bool clear_map_requested = false;
int do_map_dilate = 0;


// ------------------------------------------------------------
// Morphology helpers using S2 / DARK_NOW_REG as working register
// ------------------------------------------------------------

void dilate_reg_S2(int steps)
{
    scamp5_kernel_begin();
        SET(RN, RS, RE, RW);
        ALL();
    scamp5_kernel_end();

    for (int i = 0; i < steps; ++i)
    {
        scamp5_kernel_begin();
            DNEWS0(S6, S2);
            OR(S2, S6);
        scamp5_kernel_end();
    }
}

void erode_dreg_S2(int steps)
{
    scamp5_kernel_begin();
        SET(RN, RS, RE, RW);
        ALL();
    scamp5_kernel_end();

    for (int i = 0; i < steps; ++i)
    {
        scamp5_kernel_begin();
            NOT(S1, S2);
            DNEWS0(S6, S1);
            OR(S1, S6);
            NOT(S2, S1);
        scamp5_kernel_end();
    }
}

void closure_for_S2(int steps)
{
    dilate_reg_S2(steps);
    erode_dreg_S2(steps);
}


// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

int main()
{
    // ------------------------------------------------------------
    // Init
    // ------------------------------------------------------------

    vs_init();

    vs_on_shutdown([&]() {
        vs_post_text("M0 shutdown\n");
    });

    vs_gui_set_info(VS_M0_PROJECT_INFO_STRING);

    auto display_tx    = vs_gui_add_display("TX / selected map", 0, 0, window_size);
    auto display_debug = vs_gui_add_display("debug F", 0, window_size, window_size);
    auto display_board = vs_gui_add_display("BOARD", window_size, 0, 1);
    auto display_map   = vs_gui_add_display("MAP", window_size, 1, 1);
    auto display_ball  = vs_gui_add_display("BALL_CAND", window_size, 2, 1);


    int build_map = 0;
    vs_on_gui_update(vs_gui_add_button("build map"), [&](int32_t new_value) {
        build_map = 30;
        clear_map_requested = true;
    });


    vs_gui_move_slider(VS_GUI_FRAME_GAIN, 3);


    // ------------------------------------------------------------
    // GUI parameters
    // ------------------------------------------------------------


    int T_white = -8;
    vs_gui_add_slider("T_white", -127, 127, T_white, &T_white);

    int close_steps = 2;
    vs_gui_add_slider("close_steps", 0, 8, close_steps, &close_steps);

    int sel_x_start = 125;
    int sel_y_start = 144;
    vs_gui_add_slider("seed x", 0, 255, sel_x_start, &sel_x_start);
    vs_gui_add_slider("seed y", 0, 255, sel_y_start, &sel_y_start);

    int flood_fill_iteration = 10;
    vs_gui_add_slider("ff iterations", 0, 30, flood_fill_iteration, &flood_fill_iteration);

    int board_close_after_ff = 10;
    vs_gui_add_slider("board close after ff", 0, 30, board_close_after_ff, &board_close_after_ff);

    int edges_threshold = 0;
    vs_gui_add_slider("edges_threshold", -128, 127, edges_threshold, &edges_threshold);

    vs_gui_add_slider("do map dilate", 0, 10, do_map_dilate, &do_map_dilate);

    int T_dark = -55;
    vs_gui_add_slider("T_dark", -127, 127, T_dark, &T_dark);

    int cand_dilate = 0;
    vs_gui_add_slider("cand dilate", 0, 3, cand_dilate, &cand_dilate);

    int tx_mode = 0;
    vs_gui_add_slider("tx mode", 0, 5, tx_mode, &tx_mode);

    int show_debug_displays = 1;
    vs_gui_add_slider("show debug displays", 0, 1, show_debug_displays, &show_debug_displays);


    // ------------------------------------------------------------
    // Button: rebuild board/map
    // ------------------------------------------------------------



    // ------------------------------------------------------------
    // Clear persistent registers
    // ------------------------------------------------------------

    scamp5_kernel_begin();
        CLR(BOARD_REG);
        CLR(MAP_REG);
        CLR(DARK_NOW_REG);
        CLR(BALL_CAND_REG);
        CLR(TMP_REG);
    scamp5_kernel_end();


    // ------------------------------------------------------------
    // Frame loop
    // ------------------------------------------------------------

    while (1)
    {
        vs_frame_loop_control();


        // ------------------------------------------------------------
        // Clear map if requested
        // ------------------------------------------------------------

        if (clear_map_requested)
        {
            scamp5_kernel_begin();
                CLR(BOARD_REG);
                CLR(MAP_REG);
                CLR(DARK_NOW_REG);
                CLR(BALL_CAND_REG);
                CLR(TMP_REG);
            scamp5_kernel_end();

            clear_map_requested = false;
        }


        // ------------------------------------------------------------
        // 1. Capture image
        // ------------------------------------------------------------

        scamp5_get_image(A, C);


        // ------------------------------------------------------------
        // 2. Build board candidate from white threshold
        //
        // S0 = A > T_white
        // S2 = S0
        // ------------------------------------------------------------

        scamp5_in(F, T_white);

        scamp5_kernel_begin();
            sub(B, A, F);

            where(B);
                MOV(S0, FLAG);
            all();

            MOV(S2, S0);
        scamp5_kernel_end();


        // ------------------------------------------------------------
        // 3. Close rough white-board candidate
        // ------------------------------------------------------------

        closure_for_S2(close_steps);


        // ------------------------------------------------------------
        // 4. Flood-fill board from seed
        //
        // S0 = board candidate mask
        // S5 = seed, then flood-filled board region
        // ------------------------------------------------------------

        scamp5_kernel_begin();
            all();
            ALL();

            MOV(S0, S2);

            SET(S5);
            NOT(S6, S5);
            MOV(S5, S6);
        scamp5_kernel_end();

        DREG_load_centered_rect(S5, sel_x_start, sel_y_start, 4, 4);

        scamp5_kernel_begin();
            SET(RN, RS, RE, RW);
            SET(R0);

            MOV(R12, S5);   // flood seed
            MOV(R0, S0);    // flood mask

            for (int i = 0; i < flood_fill_iteration; ++i)
            {
                PROP0();
            }

            MOV(S5, R12);   // retrieve flood-filled result
            SET(R0);
        scamp5_kernel_end();


        // ------------------------------------------------------------
        // 5. Optional closure of final board mask
        //
        // BOARD_REG = S5 after closure
        // ------------------------------------------------------------

        scamp5_kernel_begin();
            MOV(S2, S5);
        scamp5_kernel_end();

        closure_for_S2(board_close_after_ff);

        scamp5_kernel_begin();
            MOV(BOARD_REG, S2);
        scamp5_kernel_end();


        // ------------------------------------------------------------
        // 6. Create board-masked analogue debug image F
        //
        // inside board: A
        // outside board: black
        // ------------------------------------------------------------

        scamp5_in(E, 0);

        scamp5_kernel_begin();
            WHERE(BOARD_REG);
                mov(F, A);

            all();
            ALL();

            NOT(S6, BOARD_REG);
            WHERE(S6);
                mov(F, E);

            ALL();
            all();
        scamp5_kernel_end();


        // ------------------------------------------------------------
        // 7. Build static black map while build_map > 0
        //
        // MAP_REG accumulates dark/static features inside BOARD_REG.
        //
        // This is intentionally close to your previous map path:
        //   sub(B, B, E)
        //   where(B)
        //   NOT(S6, FLAG)
        //   S6 &= BOARD_REG
        //   MAP_REG |= S6
        // ------------------------------------------------------------

        scamp5_in(E, edges_threshold);

        scamp5_kernel_begin();
            sub(B, B, E);

            where(B);
                NOT(S6, FLAG);
            all();

            AND(S6, S6, BOARD_REG);
        scamp5_kernel_end();


        if (build_map > 0)
        {
            scamp5_kernel_begin();
                OR(MAP_REG, MAP_REG, S6);
            scamp5_kernel_end();

            build_map--;

            if (build_map == 0 && do_map_dilate > 0)
            {
                scamp5_kernel_begin();
                    MOV(S2, MAP_REG);
                scamp5_kernel_end();

                dilate_reg_S2(do_map_dilate);

                scamp5_kernel_begin();
                    MOV(MAP_REG, S2);
                scamp5_kernel_end();
            }

            scamp5_kernel_begin();
                CLR(DARK_NOW_REG);
                CLR(BALL_CAND_REG);
            scamp5_kernel_end();

            vs_post_text("!clear\nbuilding map: %d frames left\n", build_map);
        }
        else
        {
            // Keep persistent map refreshed.
            scamp5_kernel_begin();
                REFRESH(MAP_REG);
                REFRESH(BOARD_REG);
            scamp5_kernel_end();


            // ------------------------------------------------------------
            // 8. Runtime binary maps
            //
            // DARK_NOW_REG:
            //      A < T_dark, restricted to board
            //
            // BALL_CAND_REG:
            //      DARK_NOW_REG & ~MAP_REG
            // ------------------------------------------------------------

            scamp5_in(E, T_dark);

            scamp5_kernel_begin();

                // DARK_NOW_REG = A < T_dark
                // Compute T_dark - A. Positive means A < T_dark.
                sub(B, E, A);

                where(B);
                    MOV(DARK_NOW_REG, FLAG);
                all();

                // Restrict current dark pixels to board.
                AND(DARK_NOW_REG, DARK_NOW_REG, BOARD_REG);

                // BALL_CAND_REG = DARK_NOW_REG & ~MAP_REG
                NOT(TMP_REG, MAP_REG);
                AND(BALL_CAND_REG, DARK_NOW_REG, TMP_REG);

            scamp5_kernel_end();


            // Optional candidate dilation.
            // Helps if ball mask is fragmented.
            for (int i = 0; i < cand_dilate; ++i)
            {
                scamp5_kernel_begin();
                    SET(RN, RS, RE, RW);
                    DNEWS0(TMP_REG, BALL_CAND_REG);
                    OR(BALL_CAND_REG, BALL_CAND_REG, TMP_REG);
                scamp5_kernel_end();
            }

            /*
            vs_post_text(
                "!clear\nmap built\n"
                "tx mode:\n"
                "0 BALL_CAND\n"
                "1 DARK_NOW\n"
                "2 MAP\n"
                "3 BOARD\n"
                "4 RAW A\n"
                "5 DEBUG F\n"
            );
            */
        }


        // ------------------------------------------------------------
        // 9. Output selected map/image
        //
        // tx_mode:
        //   0 = BALL_CAND_REG
        //   1 = DARK_NOW_REG
        //   2 = MAP_REG
        //   3 = BOARD_REG
        //   4 = raw A
        //   5 = board-masked F
        // ------------------------------------------------------------

        if (vs_gui_is_on())
        {
            if (tx_mode == 0)
            {
                scamp5_output_image(BALL_CAND_REG, display_tx);
            }
            else if (tx_mode == 1)
            {
                scamp5_output_image(DARK_NOW_REG, display_tx);
            }
            else if (tx_mode == 2)
            {
                scamp5_output_image(MAP_REG, display_tx);
            }
            else if (tx_mode == 3)
            {
                scamp5_output_image(BOARD_REG, display_tx);
            }
            else if (tx_mode == 4)
            {
                scamp5_output_image(A, display_tx);
            }
            else
            {
                scamp5_output_image(F, display_tx);
            }


            if (show_debug_displays)
            {
                scamp5_output_image(F, display_debug);
                scamp5_output_image(BOARD_REG, display_board);
                scamp5_output_image(MAP_REG, display_map);
                scamp5_output_image(BALL_CAND_REG, display_ball);
            }
        }
    }

    return 0;
}
