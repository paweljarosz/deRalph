$(document).ready(function() {

    window.manipulator_height = $('#manipulator').outerHeight();

    $('#menu').css({
        'margin-top' : function() {return -$(this).outerHeight()/2 - window.manipulator_height / 2}
    });


    $('#manipulator').on('mousedown', 'button', function() {
        var btn = $(this);
        var ring = $('#manipulator .ring');
        ring.removeClass('btn_up btn_down btn_left btn_right btn_left_up btn_right_up btn_left_down btn_right_down');
        ring.addClass(btn.prop('id'));
        $('#manipulator').addClass('active');
    }).on('mouseup', 'button', function () {
        var ring = $('#manipulator .ring');
        $('#manipulator').removeClass('active');
    });

    function windowIsMobile() {
        if( $('#battery-level-wrapper').css('display') == 'none' ) {
            window.isMobile = true;
            return true;
        }
        return false;
    }

    var gear_step_px = 35;
    var gears = 4;
    var current_gear = 3;

    $('#gear_selector').draggable({
        axis: "y",
        containment: "#gearbox",
        scroll: false,
        stop: function(event, ui) {

            var grid_y = gear_step_px;
            var elem = $( this );
            var top = parseInt(elem.css('top'));
            var cy = (top % grid_y);
            var new_top = (Math.abs(cy)+0.5*grid_y >= grid_y) ? (top - cy + (top/Math.abs(top))*grid_y) : (top - cy);
        
            ui.helper.stop(true).animate({
                top: new_top,
                opacity: 1,
            }, 200);

            
            var position = new_top;
            if (position == 0) {
                current_gear = gears;
            } else {
                current_gear = ((gear_step_px * gears) - position) / gear_step_px;
            }

            current_gear--;

            $('#gear_selector').removeClass('g1 g2 g3 stop');

            if(current_gear == 0) {
                current_gear = 'stop';
                $('#gear_selector').addClass('stop');
                $('#stop').trigger('mousedown');
            } else {
                if($('#gear_selector').hasClass('stop')) {
                    $('#stop').trigger('mouseup');
                }
                $('#gear_selector').addClass('g'+current_gear);
                $('#g'+current_gear).trigger('mousedown').delay(50).trigger('mouseup');
            }
            
            console.log(current_gear);

        }
    });
    var max_steps = 10;

    function generate_battery_level(steps) {
        var html = '';
        if(steps > max_steps) {
            steps = max_steps;
        }
        for(i=0; i < steps; i++) {
            html=html+'<div class="step">&nbsp</div>';
        }
        $('#battery-level-wrapper .inner').html(html);
    }

    $('#battery-level-wrapper .inner').append(generate_battery_level());

    function update_battery_level() {
        var voltage = parseFloat ($('#lb_bat').text() || 0);

        var max_voltage = 12;
        var min_voltage = 10;

        var max_level = max_voltage - min_voltage;
        var current_level = max_level - (max_voltage - voltage);
        var one_step = max_steps / max_level;

        var current_steps = current_level * one_step;

        generate_battery_level(Math.round(current_steps));
    }

    $('#lb_bat').on('DOMSubtreeModified', function() {
        update_battery_level();
    });

    var resize_tout;

    $(window).resize(function() {
        clearTimeout(resize_tout);
        resize_tout = setTimeout(function() {
            }, 100
        );
    });

    function init_scroller() {
        $('#console_container').mCustomScrollbar({
            theme: "light",
            scrollbarPosition: "outside",
            scrollButtons: {enable:true},
            mouseWheel: true,
            scrollInertia: 0,
            alwaysShowScrollbar: 1,
            advanced: {
                updateOnContentResize: true
            },
            callbacks: {
                onUpdate: function() {
                    $("#console_container").mCustomScrollbar("scrollTo","bottom");
                }
            }
        });
    }

    $('#info').on('click', function() {
        $('#console_container').toggleClass('hidden');
    });

    $(window).load(function () {
        init_scroller();
        update_battery_level();
    });


});