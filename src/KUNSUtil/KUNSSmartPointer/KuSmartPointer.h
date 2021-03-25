#ifndef KUNS_SMART_POINTER_H
#define KUNS_SMART_POINTER_H

#include <memory>
#include <iostream>
using namespace std;


template<class T>
class KuSmartPointer
{
public:
	KuSmartPointer();
	KuSmartPointer(T *pkPoint);
	KuSmartPointer( const KuSmartPointer &spPoint);
	~KuSmartPointer();

	operator T*() const;
	T& operator*() const;
	T* operator->() const;

	KuSmartPointer<T>& operator=(const KuSmartPointer &spPoint);
	KuSmartPointer<T>& operator=(T* pkPoint);

	bool operator== (T* pkPoint) const;
	bool operator!= (T* pkPoint) const;
	bool operator== (const KuSmartPointer &spPoint) const;
	bool operator!= (const KuSmartPointer &spPoint) const;

	int GetRefCount();
	bool IsNull();

private:
	void Inc();
	void Dec();

	T* m_pkPoint;
	int* m_pkCnt;
};





//#include "KuSmartPointer.h"

template<class T>
KuSmartPointer<T>::KuSmartPointer() : m_pkPoint(NULL), m_pkCnt(NULL)
{

}

template<class T>
KuSmartPointer<T>::KuSmartPointer(T *pkPoint)
	: m_pkPoint(pkPoint), m_pkCnt(NULL)
{
	if( m_pkPoint )
		Inc();
}

template<class T>
KuSmartPointer<T>::KuSmartPointer(const KuSmartPointer &spPoint)
	: m_pkPoint(NULL), m_pkCnt(NULL)
{
	m_pkCnt = spPoint.m_pkCnt;
	m_pkPoint = spPoint.m_pkPoint;
	if( m_pkPoint )
		Inc();
}

template<class T>
KuSmartPointer<T>::~KuSmartPointer()
{
	Dec();
}

template<class T>
KuSmartPointer<T>::operator T*() const
{
	return m_pkPoint;
}

template<class T>
T& KuSmartPointer<T>::operator*() const
{
	return *m_pkPoint;
}

template<class T>
T* KuSmartPointer<T>::operator->() const
{
	return m_pkPoint;
}

template<class T>
KuSmartPointer<T>& KuSmartPointer<T>::operator=(const KuSmartPointer<T> &spPoint)
{
	if( m_pkPoint != spPoint.m_pkPoint )
	{
		if( m_pkPoint )
			Dec();
		m_pkPoint = spPoint.m_pkPoint;
		m_pkCnt = spPoint.m_pkCnt;
		if( m_pkPoint )
			Inc();
	}

	return *this;
}

template<class T>
KuSmartPointer<T>& KuSmartPointer<T>::operator=(T* pkPoint)
{
	if( m_pkPoint != pkPoint )
	{
		if( m_pkPoint )
			Dec();
		m_pkPoint = pkPoint;
		m_pkCnt = 0;
		if( m_pkPoint )
			Inc();
	}

	return *this;
}

template<class T>
bool KuSmartPointer<T>::operator== (T* pkPoint) const
{
	return m_pkPoint == pkPoint;
}

template<class T>
bool KuSmartPointer<T>::operator!= (T* pkPoint) const
{
	return m_pkPoint != pkPoint;
}

template<class T>
bool KuSmartPointer<T>::operator== (const KuSmartPointer &spPoint) const
{
	return m_pkPoint == spPoint.m_pkPoint;
}

template<class T>
bool KuSmartPointer<T>::operator!= (const KuSmartPointer &spPoint) const
{
	return m_pkPoint != spPoint.m_pkPoint;
}

template<class T>
int KuSmartPointer<T>::GetRefCount()
{ 
	return m_pkCnt?*m_pkCnt:0;
}	

template<class T>
bool KuSmartPointer<T>::IsNull()
{
	return m_pkPoint == NULL;
}

template<class T>
void KuSmartPointer<T>::Inc()
{
	if( m_pkCnt )
		++(*m_pkCnt);
	else
		m_pkCnt = new int(1);
}

template<class T>
void KuSmartPointer<T>::Dec()
{
	if( m_pkCnt )
	{
		if( --(*m_pkCnt) <= 0 )
		{
			//	해제 되는걸 확인하기 위해 printf추가
			//printf("해제 : %d\n", *m_pkPoint);			
			delete m_pkCnt;
			m_pkCnt = 0;
			delete m_pkPoint;
			m_pkPoint = 0;
		}
	}
}

//---------------------------------------------------------------------------
#endif /*KUNS_SMART_POINTER_H*/