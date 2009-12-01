//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2009 Marianne Gagnon
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


#ifndef HEADER_WIDGET_HPP
#define HEADER_WIDGET_HPP

#include <irrlicht.h>
#include <map>

#include "guiengine/event_handler.hpp"
#include "guiengine/skin.hpp"
#include "utils/constants.hpp"
#include "utils/ptr_vector.hpp"
#include "utils/vec3.hpp"

namespace GUIEngine
{
    
    class DynamicRibbonWidget;
    
    enum WidgetType
    {
        WTYPE_NONE = -1,
        WTYPE_RIBBON,
        WTYPE_SPINNER,
        WTYPE_BUTTON,
        WTYPE_ICON_BUTTON,
        WTYPE_CHECKBOX,
        WTYPE_LABEL,
        WTYPE_SPACER,
        WTYPE_DIV,
        WTYPE_DYNAMIC_RIBBON,
        WTYPE_MODEL_VIEW,
        WTYPE_LIST,
        WTYPE_TEXTBOX
    };
    
    enum Property
    {
        PROP_ID = 100,
        PROP_PROPORTION,
        PROP_WIDTH,
        PROP_HEIGHT,
        PROP_CHILD_WIDTH,
        PROP_CHILD_HEIGHT,
        PROP_WORD_WRAP,
        PROP_GROW_WITH_TEXT, // yet unused
        PROP_X,
        PROP_Y,
        PROP_LAYOUT,
        PROP_ALIGN,
        // PROP_TEXT, // this one is a bit special, can't go along others since it's wide strings
        PROP_ICON,
        PROP_TEXT_ALIGN,
        PROP_MIN_VALUE,
        PROP_MAX_VALUE,
        PROP_MAX_WIDTH,
        PROP_MAX_HEIGHT,
        PROP_SQUARE
    };
    
    bool isWithinATextBox();
    void setWithinATextBox(bool in);
    
    /**
      * The nearly-abstract base of all widgets (not fully abstract since a bare Widget
      * can be created for the sore goal of containing children widgets in a group)
      *
      * Provides basic common functionnality, as well as providing a few callbacks
      * for children to override if they need to do something special on event.
      *
      * Each widget may have an irrlicht parent (most often used to put widgets in dialogs)
      * and also optionally one or many children.
      *
      * Each widget also has a set of properties stored in a ma (see enum above)
      */
    class Widget : public SkinWidgetContainer
    {
    protected:

        friend class EventHandler;
        friend class RibbonWidget;
        friend class Screen;
        friend class SpinnerWidget;
        friend class Skin;
        friend class DynamicRibbonWidget;
        
        
        /**
          * Can be used in children to indicate whether a widget is selected or not
          * - in widgets where it makes sense (e.g. ribbon children) and where the
          * irrLicht widget can not directly contain this state
          */
        bool m_selected[MAX_PLAYER_COUNT];
        
        /**
          * called when left/right keys pressed and focus is on widget. 
          * Returns 'EVENT_LET' if user's event handler should be notified of a change.
          * Override in children to be notified of left/right events and/or make
          * the event propagate to the user's event handler.
          */
        virtual EventPropagation rightPressed(const int playerID) { return EVENT_BLOCK; }
        virtual EventPropagation leftPressed(const int playerID) { return EVENT_BLOCK; }
        
        /** used when you set eventSupervisors - see m_event_handler explainations below
            called when one of a widget's children is hovered.
            Returns 'true' if main event handler should be notified of a change. */
        virtual EventPropagation mouseHovered(Widget* child, const int playerID) { return EVENT_BLOCK; }
        
        /** override in children if you need to know when the widget is focused. return whether to block event */
        virtual EventPropagation focused(const int playerID) { setWithinATextBox(false); return EVENT_LET; }
        
        /** override in children if you need to know when the widget is unfocused. */
        virtual void unfocused(const int playerID) { }
        
        /**
          * The XML loader stored coords in their raw string form inside this widget.
          * This method parses the strings. Most notably, expands coords relative to parent
          * and calculates percentages.
          */
        void readCoords(Widget* parent=NULL);
        
        /**
         * An irrlicht parent (most often used to put widgets in dialogs)
         */
        irr::gui::IGUIElement* m_parent;
        
        /**
         * Receives as string the raw property value retrieved from XML file.
         * Will try to make sense of it, as an absolute value or a percentage.
         *
         * Return values :
         *     Will write to either absolute or percentage, depending on the case.
         *     Returns false if couldn't convert to either
         */
        static bool convertToCoord(std::string& x, int* absolute, int* percentage);
        
        /**
         * IrrLicht widget created to represent this object.
         */
        irr::gui::IGUIElement* m_element;
        
        
        // FIXME... i forgot the m_ everywhere ... XD

        /** numerical ID used by irrLicht to identify this widget
         * (not the same as the string identificator specified in the XML file)
         */
        int id;
        
        /** Usually, only one widget at a time can be focused. There is however a special case where all
            players can move through the screen. This variable will then be used as a bitmask to contain
            which players beyong player 1 have this widget focused. */
        bool m_player_focus[MAX_PLAYER_COUNT];

        bool m_reserve_id;
                
    public:
        /**
         * This is set to NULL by default; set to something else in a widget to mean
         * that events happening on this widget should also be passed to m_event_handler->transmitEvent,
         * which is usually the parent analysing events from its children.
         * This is especially useful with logical widgets built with more than
         * one irrlicht widgets (e.g. Spinner, Ribbon)
         */
        Widget* m_event_handler;
        
        
        /** Instead of searching for widget IDs smaller/greater than that of this object, navigation
            through widgets will start from these IDs (if they are set). */
        int m_tab_down_root;
        int m_tab_up_root;
        
        /** Coordinates of the widget once added (the difference between those x/h and PROP_WIDTH/PROP_HEIGHT is
            that the props are read in raw form from the XML file; PROP_WIDTH can then be e.g. "10%" and w,
            once the widget is added, will be e.g. 80.) */
        int x, y, w, h;
        
        /** Whether to show a bounding box around this widget (used for sections) */
        bool m_show_bounding_box;
        
        /** Used in two cases :
            1) For 'placeholder' divisions; at the time the layout is created, there is nothing to
               place there yet, but we know there eventually will. So in this case pass 'true' to the
               Widget constructor and it will reserve a widget ID and store it here.
            2) Theorically, in 'add()', derived widgets should checked if this value is set, and use
               it instead of creating a new ID if it is. In practice, it's not widely implemented (FIXME) */
        int m_reserved_id;
        
        Widget(bool reserve_id = false);
        virtual ~Widget();
        
        /**
          * Get the underlying irrLicht GUI element, casted to the right type.
          */
        template<typename T> T* getIrrlichtElement()
        {
        #if HAVE_RTT
            T* out = dynamic_cast<T*>(m_element);
            return out;
        #else
            return static_cast<T*>(m_element);
        #endif
        }

        template<typename T> const T* getIrrlichtElement() const
        {
            #if HAVE_RTT
                T* out = dynamic_cast<T*>(m_element);
                return out;
            #else
                return static_cast<T*>(m_element);
            #endif
        }

        irr::gui::IGUIElement* getIrrlichtElement() { return m_element; }

        void setParent(irr::gui::IGUIElement* parent);
        
        /**
          * If this widget has any children, they go here. Children can be either
          * specified in the XML file (e.g. Ribbon or Div children), or can also
          * be created automatically for logical widgets built with more than
          * one irrlicht widgets (e.g. Spinner)
          */
        ptr_vector<Widget> m_children;
        
        /** Type of this widget */
        WidgetType m_type;
        
        /** A map that holds values for all specified widget properties (in the XML file)*/
        std::map<Property, std::string> m_properties;
        
        /** PROP_TEXT is a special case : since it can be transalted it can't go in the map above, which
            uses narrow strings */
        irr::core::stringw m_text;
        
        static void resetIDCounters();
        
        /**
         * \param playerID ID of the player you want to set/unset focus for, starting from 0
         */
        void setFocusForPlayer(const int playerID);
        
        /**
         * \param playerID ID of the player you want to set/unset focus for, starting from 0
         */
        bool isFocusedForPlayer(const int playerID);
        
        /** Internal method, do not call it. Call the functions in GUIEngine instead to unset focus. */
        void unsetFocusForPlayer(const int playerID);
        
        /**
          * Call to resize/move the widget. Not all widgets can resize gracefully.
          */
        virtual void move(const int x, const int y, const int w, const int h);
        
        
        bool isSelected(const int playerID) const { return m_selected[playerID]; }
        
        bool isSameIrrlichtWidgetAs(const Widget* ref) const { return m_element == ref->m_element; }
        
        /**
         * These methods provide new unique IDs each time you call them.
         * Since IDs are used to determine tabbing order, "non-tabbable"
         * objects are being given very different IDs so that they don't interfere.
         */
        static int getNewID();
        static int getNewNoFocusID();
        
        /**
         * Override in children to possibly receive updates (you may need to register to
         * them first)
         */
        virtual void update(float delta) { }
        
        /** All widgets, including their parents (m_event_handler) will be notified on event through
         this call. Must return whether main (GUI engine user) event callback should be notified or not.
         Note that in the case of a hierarchy of widgets (with m_event_handler), only the topmost widget
         of the chain decides whether the main handler is notified; return value is not read for others. */
        virtual EventPropagation transmitEvent(Widget* w, std::string& originator, const int playerID) { return EVENT_LET; }
        
        /**
         * Create and add the irrLicht widget(s) associated with this object.
         * Call after Widget was read from XML file and laid out.
         */
        virtual void add();
        
        /**
          * Called when irrLicht widgets cleared. Forget all references to them, they're no more valid.
          */
        virtual void elementRemoved();
    };

    
}
#endif
