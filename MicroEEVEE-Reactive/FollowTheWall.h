/* 
 * File:   FollowTheWall.h
 * Author: bluemoon
 *
 * Created on October 28, 2014, 9:09 PM
 */

#ifndef FOLLOWTHEWALL_H
#define	FOLLOWTHEWALL_H

#ifdef	__cplusplus
extern "C" {
#endif

    void prep_ftw();
    char* ftw_getName();
    void ftw_execute();
    bool ftw_isPossible();
    
    void atBeacon();


#ifdef	__cplusplus
}
#endif

#endif	/* FOLLOWTHEWALL_H */

